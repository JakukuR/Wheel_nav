"""Curiosity and wander choices. Pure functions over the session memory."""
from dataclasses import dataclass
import math
import numpy as np
from .memory import wrap


@dataclass
class Weights:
    # Curiosity
    novelty: float = .6
    rarity: float = .3
    salience: float = .4
    salience_tau_s: float = 60.
    distance_cost: float = .12      # per metre
    turn_cost: float = .15          # per pi radians
    revisit_penalty: float = .3
    revisit_tau_s: float = 300.
    curiosity_threshold: float = .35
    curiosity_temperature: float = .12
    curiosity_max_range_m: float = 3.5
    stale_after_s: float = 180.
    # Wander
    reach: float = .35
    unvisited: float = .35
    wander_turn: float = .2
    wide: float = .1
    homing: float = .3
    wander_noise: float = .05
    min_reach_m: float = 1.0
    leg_min_m: float = 1.0
    leg_max_m: float = 2.0
    stop_short_m: float = .35
    # Connected-region wander (local grid)
    region_min_m: float = 1.2
    region_max_m: float = 2.8
    region_clearance_m: float = .32
    region_cell_m: float = .1
    region_blind_m: float = .9
    momentum: float = .25
    openness: float = .2
    frontier: float = .25
    region_temperature: float = .08


def object_scores(memory, pose, now, start_xy, boundary_m, w=Weights(), exclude=('person',)):
    """[(score, instance, parts)] for every instance allowed to be chosen now."""
    rows = []
    for inst in memory.instances:
        if inst.label in exclude or now < inst.cooldown_until or now-inst.last_seen > w.stale_after_s:
            continue
        delta = inst.xy-pose.xy(); distance = float(np.linalg.norm(delta))
        if distance > w.curiosity_max_range_m or np.linalg.norm(inst.xy-start_xy) > boundary_m:
            continue
        bearing = abs(wrap(math.atan2(delta[1], delta[0])-pose.yaw))
        novelty = w.novelty/(1.+2.*inst.visits)*(.5+.5/(1.+inst.sightings/10.))
        rarity = w.rarity/max(1, memory.label_count(inst.label))
        decay = lambda t: math.exp(-max(0., now-t)/w.salience_tau_s)
        salience = w.salience*max(decay(inst.first_seen), decay(inst.moved_at), decay(inst.touched_at))
        cost = w.distance_cost*distance+w.turn_cost*bearing/math.pi
        penalty = (w.revisit_penalty*math.exp(-(now-inst.cooldown_until)/w.revisit_tau_s)
                   if inst.visits or inst.failures else 0.)
        score = novelty+rarity+salience-cost-penalty
        rows.append((score, inst, dict(novelty=round(novelty, 3), rarity=round(rarity, 3),
            salience=round(salience, 3), cost=round(cost, 3), penalty=round(penalty, 3))))
    return rows


def choose_object(rows, rng, w=Weights()):
    """Softmax over candidates above the threshold; None means go wandering."""
    rows = [row for row in rows if row[0] >= w.curiosity_threshold]
    if not rows:
        return None
    scores = np.array([row[0] for row in rows])
    p = np.exp((scores-scores.max())/w.curiosity_temperature); p /= p.sum()
    return rows[int(rng.choice(len(rows), p=p))]


def wander_goal(sectors, visits, pose, now, start_xy, boundary_m, rng, w=Weights(), max_age=60.,
                blocked=()):
    """Pick a sector and a 1-2 m point on it; None when nothing is open enough."""
    best = None
    home = start_xy-pose.xy(); away = float(np.linalg.norm(home))
    for index in sectors.fresh(now, max_age):
        if index in blocked or sectors.reach[index] < w.min_reach_m:
            continue
        heading = sectors.heading(index)
        length = min(rng.uniform(w.leg_min_m, w.leg_max_m), sectors.reach[index]-w.stop_short_m)
        if length < w.leg_min_m-.2:
            continue
        goal = pose.xy()+length*np.array([math.cos(heading), math.sin(heading)])
        if np.linalg.norm(goal-start_xy) > boundary_m:
            continue
        score = (w.reach*min(sectors.reach[index], 3.)/3.+w.unvisited*(1.-visits.level(goal, now))
                 -w.wander_turn*abs(wrap(heading-pose.yaw))/math.pi+w.wide*sectors.wide[index]
                 +rng.normal(0., w.wander_noise))
        if away > .7*boundary_m:
            score += w.homing*math.cos(wrap(heading-math.atan2(home[1], home[0])))
        if best is None or score > best[0]:
            best = (score, index, goal)
    return best


def grid_ray_measure(grid, resolution, origin_xy, origin_yaw, offsets=(-.25, 0., .25),
                     wide_offsets=(-.45, .45), max_range=3., blind_m=.9, step=.05):
    """Ray-march an OccupancyGrid given in the robot frame (robot at its origin).

    Reach stops at an occupied cell, or at unknown space once known-free space
    was entered (the camera's near blind zone in front of the body is allowed).
    The body is covered by parallel rays at +-0.25 m; "wide" counts how many of
    the outer rays at +-0.45 m stay open as far.
    """
    grid = np.asarray(grid); height, width = grid.shape
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)

    def cell(point):
        dx, dy = point[0]-origin_xy[0], point[1]-origin_xy[1]
        gx, gy = (c*dx+s*dy)/resolution, (-s*dx+c*dy)/resolution
        i, j = int(math.floor(gy)), int(math.floor(gx))
        return grid[i, j] if 0 <= i < height and 0 <= j < width else -1

    def ray(bearing, offset):
        direction = np.array([math.cos(bearing), math.sin(bearing)])
        normal = np.array([-direction[1], direction[0]])*offset
        known = False; distance = 0.
        while distance < max_range:
            value = cell(normal+direction*(distance+step))
            if value >= 50:
                return distance
            if value < 0:
                if known or distance+step > blind_m:
                    return distance
            else:
                known = True
            distance += step
        return max_range

    def measure(bearing):
        reach = min(ray(bearing, offset) for offset in offsets)
        if reach <= 0.:
            return (0., 0.)
        wide = np.mean([ray(bearing, offset) >= min(reach, 1.5)-1e-6 for offset in wide_offsets])
        return (reach, float(wide))
    return measure


@dataclass
class GridSnapshot:
    """OccupancyGrid in the robot frame (robot at its origin) plus the odom pose it was taken at."""
    grid: np.ndarray
    resolution: float
    origin_xy: tuple
    origin_yaw: float
    pose_xy: np.ndarray
    pose_yaw: float


def _blocks(snapshot, cell_m):
    g = np.asarray(snapshot.grid); s = max(1, int(round(cell_m/snapshot.resolution)))
    h, w = g.shape[0]//s*s, g.shape[1]//s*s
    g = g[:h, :w].reshape(h//s, s, w//s, s)
    occupied = (g >= 50).any(axis=(1, 3))
    free = ((g == 0).mean(axis=(1, 3)) >= .5) & ~occupied
    res = snapshot.resolution*s
    jj, ii = np.meshgrid(np.arange(w//s), np.arange(h//s))
    local = np.stack(((jj+.5)*res, (ii+.5)*res), axis=-1)
    c, sn = math.cos(snapshot.origin_yaw), math.sin(snapshot.origin_yaw)
    robot = local@np.array([[c, sn], [-sn, c]])+np.asarray(snapshot.origin_xy)
    return occupied, free, robot, res


def connected_goal(snapshot, visits, pose, now, start_xy, boundary_m, rng, travel_heading=None, w=Weights()):
    """A 1.2-2.8 m goal in the free region connected to the robot on the local grid.

    Cells need body clearance (radius + margin) from observed obstacles; unknown
    cells only count inside the camera's near blind zone. Reachability is the
    geodesic distance through the region, so goals behind walls or around long
    detours are dropped. Scoring favours unvisited, open space, carrying on in
    the current walking direction and edges towards unseen space; the choice is
    a softmax sample, not the single best cell.
    """
    from scipy import ndimage
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import dijkstra
    occupied, free, robot, res = _blocks(snapshot, w.region_cell_m)
    clear = ndimage.distance_transform_edt(~occupied)*res
    dist = np.linalg.norm(robot, axis=-1)
    unknown = ~occupied & ~free
    ok = (clear >= w.region_clearance_m) & (free | (unknown & (dist <= w.region_blind_m)))
    labels, _ = ndimage.label(ok)
    start = np.unravel_index(int(np.argmin(np.where(ok, dist, np.inf))), ok.shape)
    if not ok[start] or dist[start] > .4:
        return None
    region = labels == labels[start]
    idx = np.flatnonzero(region.reshape(-1)); n = region.shape[1]
    pos = -np.ones(region.size, dtype=int); pos[idx] = np.arange(len(idx))
    ii, jj = np.divmod(idx, n); src, dst, cost = [], [], []
    flat = region.reshape(-1)
    for di, dj in ((1, 0), (0, 1), (1, 1), (1, -1)):
        ti, tj = ii+di, jj+dj
        valid = (ti >= 0) & (ti < region.shape[0]) & (tj >= 0) & (tj < n)
        a = np.flatnonzero(valid); b = ti[a]*n+tj[a]
        good = flat[b]
        if di and dj:
            good &= flat[ii[a]*n+tj[a]] & flat[ti[a]*n+jj[a]]
        a, b = a[good], pos[b[good]]
        step = math.hypot(di, dj)*res
        src += [a, b]; dst += [b, a]; cost += [np.full(len(a), step)]*2
    if not src:
        return None
    graph = coo_matrix((np.concatenate(cost), (np.concatenate(src), np.concatenate(dst))), shape=(len(idx), len(idx))).tocsr()
    geo = dijkstra(graph, indices=int(pos[start[0]*n+start[1]]))
    r = dist.reshape(-1)[idx]
    keep = (np.isfinite(geo) & (r >= w.region_min_m) & (r <= w.region_max_m) & (geo <= np.maximum(1.5*r, r+.6)))
    if not keep.any():
        return None
    cells = idx[keep]
    if len(cells) > 400:
        cells = rng.choice(cells, 400, replace=False)
    window = 2*int(round(.5/res))+1
    near_unknown = ndimage.uniform_filter(unknown.astype(float), size=window).reshape(-1)
    c, sn = math.cos(snapshot.pose_yaw), math.sin(snapshot.pose_yaw)
    local = robot.reshape(-1, 2)[cells]
    odom = local@np.array([[c, sn], [-sn, c]])+np.asarray(snapshot.pose_xy)
    home = start_xy-pose.xy(); away = float(np.linalg.norm(home))
    rows = []
    for k, cell in enumerate(cells):
        goal = odom[k]
        if np.linalg.norm(goal-start_xy) > boundary_m:
            continue
        delta = goal-pose.xy(); heading = math.atan2(delta[1], delta[0])
        parts = dict(unvisited=w.unvisited*(1.-visits.level(goal, now)),
                     openness=w.openness*min(float(clear.reshape(-1)[cell]), 1.),
                     frontier=w.frontier*min(1., 2.*float(near_unknown[cell])),
                     turn=-w.wander_turn*abs(wrap(heading-pose.yaw))/math.pi,
                     momentum=0. if travel_heading is None else w.momentum*math.cos(wrap(heading-travel_heading)))
        if away > .7*boundary_m:
            parts['homing'] = w.homing*math.cos(wrap(heading-math.atan2(home[1], home[0])))
        rows.append((sum(parts.values())+rng.normal(0., w.wander_noise), goal, parts))
    if not rows:
        return None
    scores = np.array([row[0] for row in rows])
    p = np.exp((scores-scores.max())/w.region_temperature); p /= p.sum()
    score, goal, parts = rows[int(rng.choice(len(rows), p=p))]
    return score, goal, {k: round(v, 3) for k, v in parts.items()}
