"""Edge-point detour reference for the sampled MPC (prototype).

When the straight route to the target is blocked within a few metres, pick a
point on the edge of the inflated free space (safe cells bordering unsafe ones,
like Avoid-MPC's dilated-edge tree) on the left or the right, and route
robot -> edge point -> target. The MPC then samples around that route; it still
owns collision checking, speed and curvature. A chosen side is held until the
straight route is clear again, so the reference does not flip left/right.
"""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from .common import wrap

# Lattice spacing of the grid-route fallback (map cells are merged up to this).
GRID_ROUTE_CELL_M = .1


class DetourPlanner:
    def __init__(self, *, probe_m=3., near_m=.4, far_m=3., max_bearing=1.9, sample_m=.05,
                 candidates_per_side=40, switch_ratio=1.3, onward_m=.8, standoff_m=.15):
        self.probe_m, self.near_m, self.far_m = probe_m, near_m, far_m
        self.max_bearing, self.sample_m = max_bearing, sample_m
        self.candidates_per_side, self.switch_ratio = candidates_per_side, switch_ratio
        self.onward_m, self.standoff_m = onward_m, standoff_m
        self.side = None
        self.last = None
        self.look_hold = 0
        # 'line': detour when robot->target is blocked; 'path': only when the
        # followed reference (the person's trail) is blocked for the body.
        self.trigger = 'line'
        # Exit hysteresis: the straight route must stay clear for this many plans
        # (about 0.5 s) before a detour ends; meanwhile the waypoint is kept.
        # 17:47 J02: a one-frame clear check reset the side 6 times in 3 s while the
        # robot drove at the obstacle.
        self.exit_cycles = 12
        self.clear_count = 0
        self.held = None

    def _clear(self, a, b, local_map, need, body):
        length = float(np.linalg.norm(b - a))
        points = a + np.linspace(0., 1., max(2, int(length / self.sample_m) + 1))[:, None] * (b - a)
        return bool((local_map.query(points) >= need).all() and
                    (local_map.query(points, unknown=True) >= body).all())

    def _clear_many(self, a, b, local_map, need, body):
        """Vectorised _clear for segments a[i]->b[i] (sampled at <= sample_m)."""
        if not len(a):
            return np.zeros(0, dtype=bool)
        n = max(2, int(float(np.max(np.linalg.norm(b - a, axis=1))) / self.sample_m) + 1)
        u = np.linspace(0., 1., n)[None, :, None]
        points = (a[:, None, :] + u * (b - a)[:, None, :]).reshape(-1, 2)
        ok = (local_map.query(points) >= need) & (local_map.query(points, unknown=True) >= body)
        return ok.reshape(len(a), n).all(axis=1)

    def plan(self, pose, target, local_map, c, reference=None):
        """Return a polyline (robot, edge point, target) or None when not blocked."""
        p = np.asarray(pose[:2], dtype=float); target = np.asarray(target, dtype=float)
        delta = target - p; distance = float(np.linalg.norm(delta))
        need = c.radius + c.margin
        self.last = dict(active=False)
        if distance < .5:
            self.side = None; return None
        if self.trigger == 'path' and reference is not None and len(reference) >= 2:
            if self._path_clear(p, np.asarray(reference, dtype=float), local_map, c.radius):
                self.side = None; return None
        probe = p + delta * min(1., self.probe_m / distance)
        # Blocked means the body itself would hit something on the straight route;
        # merely entering the soft margin band (narrow corridor) is not a reason to detour.
        if self._clear(p, probe, local_map, c.radius, 0.):
            self.clear_count += 1
            held = self.held
            if (held is not None and self.clear_count < self.exit_cycles
                    and np.linalg.norm(held - p) > .25 and self._clear(p, held, local_map, c.radius, 0.)):
                self.last = dict(active=True, side='left' if self.side and self.side > 0 else 'right',
                                 waypoint=held.tolist(), holding=True)
                return np.vstack((p, held, target))
            self.side = None; self.held = None; return None
        self.clear_count = 0
        best = {}
        # Prefer waypoints with an extra standoff; narrow gaps fall back to the plain margin.
        for standoff in (self.standoff_m, 0.):
            best = self._candidates(p, target, delta, distance, local_map, c, need, standoff)
            if best:
                break
        if not best:
            # Multi-segment way round (e.g. step aside, then forward): grid search.
            route = self._grid_route(p, target, local_map, c)
            if route is not None:
                self.look_hold = 0
                self.held = np.asarray(route[1], dtype=float).copy()
                self.last = dict(active=True, side='grid', waypoint=route[1].tolist(), route=route.tolist())
                return route
            # Nothing reachable yet: prefer looking towards a side where an opening
            # is already visible but the way there is still unseen.
            seen = self._candidates(p, target, delta, distance, local_map, c, need, 0., reachable=False)
            return self._look(p, delta, distance, local_map, seen)
        self.look_hold = 0
        choice = min(best, key=lambda s: best[s][0])
        if self.side in best and best[self.side][0] <= self.switch_ratio * best[choice][0]:
            choice = self.side
        self.side = choice
        waypoint = best[choice][1]
        self.held = np.asarray(waypoint, dtype=float).copy()
        self.last = dict(active=True, side='left' if choice > 0 else 'right', waypoint=waypoint.tolist(),
                         score=best[choice][0])
        return np.vstack((p, waypoint, target))

    def _candidates(self, p, target, delta, distance, local_map, c, need, standoff, reachable=True):
        # Waypoint cells keep an extra standoff beyond body plus margin (a further
        # dilation, as in Avoid-MPC), inside known free space.
        offset = local_map.res / np.sqrt(2)
        safe = ((local_map.dist - offset >= need + standoff) &
                (local_map.free_dist - offset >= c.radius + standoff))
        padded = np.pad(safe, 1, constant_values=False); unsafe_near = np.zeros_like(safe)
        n = safe.shape[0]
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx or dy:
                    unsafe_near |= ~padded[1 + dx:1 + dx + n, 1 + dy:1 + dy + n]
        edge = safe & unsafe_near
        points = (local_map.cells[edge] + .5) * local_map.res
        if not len(points):
            return {}
        rel = points - p; r = np.linalg.norm(rel, axis=1)
        bearing = wrap(np.arctan2(rel[:, 1], rel[:, 0]) - np.arctan2(delta[1], delta[0]))
        keep = (r > self.near_m) & (r < self.far_m) & (np.abs(bearing) < self.max_bearing)
        points, rel, r, bearing = points[keep], rel[keep], r[keep], bearing[keep]
        side = np.sign(delta[0] * rel[:, 1] - delta[1] * rel[:, 0])
        score = r + np.linalg.norm(target - points, axis=1) + .5 * np.abs(bearing)
        best = {}
        for s in (1., -1.):
            idx = np.flatnonzero(side == s)
            order = idx[np.argsort(score[idx])][:self.candidates_per_side]
            if not len(order):
                continue
            if reachable:
                # A useful edge point is past the obstacle corner: reachable now, and
                # the way onward towards the target is also open for a short stretch.
                # Onward = the robot-to-target heading, which runs along the obstacle
                # side instead of cutting back into it. All candidates of a side are
                # checked in one map query; the best-scored passing one wins.
                w = points[order]
                onward = np.minimum(self.onward_m, np.linalg.norm(target - w, axis=1))
                ahead = w + (delta / distance)[None, :] * onward[:, None]
                ok = (self._clear_many(np.repeat(p[None, :], len(w), axis=0), w, local_map, need, c.radius) &
                      self._clear_many(w, ahead, local_map, need, c.radius))
                passing = np.flatnonzero(ok)
                if not len(passing):
                    continue
                order = order[passing[:1]]
            i = order[0]
            best[s] = (float(score[i]), points[i])
        return best

    def _look(self, p, delta, distance, local_map, seen=None):
        """No reachable edge: turn to look at the side with more unseen space nearby.

        The forward camera leaves the flanks unknown, and the unknown-space rule
        forbids driving there, so the only useful move is to rotate and observe.
        The side is held like a detour side to avoid swinging back and forth.
        """
        centers = (local_map.cells + .5) * local_map.res
        rel = centers - p; r = np.linalg.norm(rel, axis=-1)
        bearing = wrap(np.arctan2(rel[..., 1], rel[..., 0]) - np.arctan2(delta[1], delta[0]))
        unseen = ~local_map.known & (r > .3) & (r < 2.) & (np.abs(bearing) > .4) & (np.abs(bearing) < 1.9)
        counts = {s: int(np.count_nonzero(unseen & (np.sign(bearing) == s))) for s in (1., -1.)}
        if not any(counts.values()) and not seen:
            self.last = dict(active=True, reason='no_clear_edge_nothing_to_look_at'); return None
        if seen:
            preferred = min(seen, key=lambda s: seen[s][0])
        else:
            preferred = max(counts, key=counts.get)
        side = self.side if self.side in counts and counts[self.side] else preferred
        if self.look_hold >= 10:
            # Looked this way for ~1 s without finding a way round: look the other way.
            side = -side if counts.get(-side) or (seen and -side in seen) else side
            self.look_hold = 0
        self.side = side
        angle = np.arctan2(delta[1], delta[0]) + side * 1.2
        look = p + .6 * np.array([np.cos(angle), np.sin(angle)])
        self.last = dict(active=True, side='left' if side > 0 else 'right', looking=True, waypoint=look.tolist())
        return np.vstack((p, look))

    def look_reached(self, reached):
        """Count control cycles spent already facing the look direction."""
        self.look_hold = self.look_hold + 1 if reached else 0
        return reached

    def _grid_route(self, p, target, local_map, c, window_m=3.5, clearance_weight=2.):
        """Dijkstra over known-free cells with room for the body, in a window round the robot.

        Ends at the reachable cell closest to the target (the target itself may
        sit in unknown or occupied cells). Cost = length + clearance penalty, so
        the route keeps to the middle of gaps. Returns a thinned polyline or None.
        """
        res = local_map.res; offset = res / np.sqrt(2)
        centers = (local_map.cells + .5) * res
        inside = np.all(np.abs(centers - p) <= window_m, axis=-1)
        clear = local_map.dist - offset - c.radius
        ok = inside & (clear >= 0.) & (local_map.free_dist - offset >= c.radius)
        # Search on a ~0.1 m lattice: a block is usable only if all its fine cells
        # are (conservative), clearance is the block minimum. A 0.05 m map would
        # otherwise make this Dijkstra 4x larger than its 0.1 m design.
        s = max(1, int(round(GRID_ROUTE_CELL_M / res)))
        if s > 1:
            m = ok.shape[0] // s * s
            block = lambda a: a[:m, :m].reshape(m // s, s, m // s, s, *a.shape[2:])
            ok = block(ok).all(axis=(1, 3))
            clear = block(clear).min(axis=(1, 3))
            centers = block(centers).mean(axis=(1, 3))
            res = res * s
        idx = np.flatnonzero(ok.reshape(-1))
        if len(idx) < 4:
            return None
        n = ok.shape[1]; flat = ok.reshape(-1); pos = -np.ones(flat.size, dtype=int); pos[idx] = np.arange(len(idx))
        ii, jj = np.divmod(idx, n); src, dst, cost = [], [], []
        penalty = clearance_weight * np.maximum(c.margin + .15 - clear.reshape(-1)[idx], 0.)
        for di, dj in ((1, 0), (0, 1), (1, 1), (1, -1)):
            ti, tj = ii + di, jj + dj
            valid = (ti >= 0) & (ti < ok.shape[0]) & (tj >= 0) & (tj < n)
            a = np.flatnonzero(valid); b = ti[a] * n + tj[a]
            good = flat[b]
            if di and dj:  # no diagonal corner cutting
                good &= flat[ii[a] * n + tj[a]] & flat[ti[a] * n + jj[a]]
            a, b = a[good], pos[b[good]]
            step = np.hypot(di, dj) * res
            w = step + .5 * step * (penalty[a] + penalty[b])
            src += [a, b]; dst += [b, a]; cost += [w, w]
        if not src:
            return None
        graph = coo_matrix((np.concatenate(cost), (np.concatenate(src), np.concatenate(dst))),
                           shape=(len(idx), len(idx))).tocsr()
        cells_xy = centers.reshape(-1, 2)[idx]
        start = int(np.argmin(np.linalg.norm(cells_xy - p, axis=1)))
        if np.linalg.norm(cells_xy[start] - p) > 2 * res:
            return None
        distance, pred = dijkstra(graph, indices=start, return_predecessors=True)
        reach = np.isfinite(distance)
        to_target = np.linalg.norm(cells_xy - target, axis=1)
        if not reach.any():
            return None
        # End at a reachable cell that is past the obstacle: from there the way on
        # towards the target is open for a stretch. The closest cell in front of a
        # wall is a dead end, not a way round.
        order = np.argsort(np.where(reach, to_target + .05 * distance, np.inf))
        goal = None
        for g in order[:300]:
            if not reach[g] or to_target[g] > np.linalg.norm(target - p) - .3:
                break
            w = cells_xy[g]; onward = target - w; length = float(np.linalg.norm(onward))
            ahead = w + onward * min(1., 1. / max(length, 1e-9))
            if self._clear(w, ahead, local_map, c.radius, c.radius):
                goal = int(g); break
        if goal is None:
            return None
        chain = [goal]
        while chain[-1] != start:
            chain.append(int(pred[chain[-1]]))
        route = cells_xy[chain[::-1]]
        thin = max(2, 4 // s)  # waypoints about every 0.2 m, as on the fine grid
        keep = [0] + list(range(thin, len(route) - 1, thin)) + [len(route) - 1]
        route = np.vstack((p, route[keep[1:]], target))
        return route

    def _path_clear(self, p, path, local_map, need):
        """Body clearance along the first probe_m of a polyline starting at the robot."""
        pts = np.vstack((p, path)); seg = np.diff(pts, axis=0); length = np.linalg.norm(seg, axis=1)
        travelled = 0.
        for a, d, l in zip(pts[:-1], seg, length):
            if l < 1e-6: continue
            take = min(l, self.probe_m - travelled)
            b = a + d * (take / l)
            if not self._clear(a, b, local_map, need, 0.):
                return False
            travelled += take
            if travelled >= self.probe_m - 1e-6: break
        return True
