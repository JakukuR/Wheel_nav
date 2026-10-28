"""Connected local-map endpoint selection for finite object approach tasks."""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.spatial import cKDTree
from .common import rotation

# The former object approach used this rectangular body. A circumscribed
# circle is used for path sweeps, including rotation; gap reporting uses the box.
HALF_LENGTH = HALF_WIDTH = .24
SWEEP_RADIUS = float(np.hypot(HALF_LENGTH, HALF_WIDTH))
DESIRED_GAP = .15
# Route search margin around the robot-object box.
PLAN_MARGIN_M = 1.2
# Route graph lattice spacing.
ROUTE_CELL_M = .1


def surface_gap(pose, surface):
    local = (np.asarray(surface)-np.asarray(pose[:2])) @ rotation(pose[2])
    delta = np.maximum(np.abs(local)-[HALF_LENGTH, HALF_WIDTH], 0.)
    return float(np.min(np.linalg.norm(delta, axis=1)))


def clear_segments(mapping, points, radius=SWEEP_RADIUS, margin=.10):
    """Conservative samples: add half sample spacing to the clearance bound.

    Only observed obstacles block the route. Unknown cells (e.g. the depth
    camera's near blind zone in front of the robot) are treated like in person
    approach: the MPC refuses unknown space only over its near horizon, so the
    robot creeps forward, observes, and the route is re-checked every update.
    """
    return mapping.query(points) >= radius+margin+.014


class ApproachGoal:
    def __init__(self):
        self.plan = None
        self.planned_at = -np.inf
        self.center = None
        self.generation = 0
        self.stable_since = None

    def select(self, pose, surface, mapping, now):
        surface = np.unique(np.round(np.asarray(surface, dtype=float).reshape(-1,2)/.025)*.025,axis=0)
        if len(surface) < 3 or not np.isfinite(surface).all():
            self.plan = None
            return None
        tree = cKDTree(surface)
        center = np.median(surface, axis=0)
        # A bounded refresh interval avoids solving a whole graph every MPC step.
        # Every reused route is still rechecked against the latest map and surface.
        if (self.plan is not None and now-self.planned_at < .4 and
                np.linalg.norm(center-self.center) < .06):
            path = self.plan['path']
            near = int(np.argmin(np.linalg.norm(path-pose[:2],axis=1)))
            suffix = np.vstack((pose[:2],path[near:]))
            samples = self._samples(suffix)
            if (np.all(clear_segments(mapping,samples)) and
                    tree.query(samples)[0].min() >= SWEEP_RADIUS+DESIRED_GAP):
                self.plan['path'] = suffix
                return self._diagnostic(pose,surface,now)
        old = self.plan
        self.planned_at, self.center = now, center
        self.plan = None
        # Route nodes stay on a ~0.1 m lattice even on a finer map: clearance is
        # still read from the fine map, but the graph keeps a bounded size.
        stride = max(1, int(round(ROUTE_CELL_M/mapping.res)))
        cells = mapping.cells[::stride, ::stride]; res = mapping.res*stride
        xy = ((cells+.5)*mapping.res).reshape(-1,2)
        n = cells.shape[0]
        # Plan only inside the box spanning robot and object plus a margin: the
        # whole map (25600 cells at 0.05 m) cost ~0.3 s per replan on the Orin.
        low = np.minimum(pose[:2], center)-PLAN_MARGIN_M; high = np.maximum(pose[:2], center)+PLAN_MARGIN_M
        window = np.flatnonzero(np.all((xy >= low) & (xy <= high), axis=1))
        valid = np.zeros(len(xy), dtype=bool)
        if len(window):
            distance = tree.query(xy[window])[0]
            # Full circular sweep is conservative for the rectangular body at any yaw.
            # Surface points are also independent obstacles, even if map log odds lag.
            valid[window] = (clear_segments(mapping,xy[window]) & (distance >= SWEEP_RADIUS+DESIRED_GAP+.014))
        valid = valid.reshape(n,n)
        indices = np.arange(n*n).reshape(n,n)
        srcs=[]; dsts=[]; costs=[]
        # Four undirected edges form an 8-connected grid, with no diagonal cuts.
        for dx,dy in ((1,0),(0,1),(1,1),(1,-1)):
            xs=slice(0,n-dx); xt=slice(dx,n)
            ys=slice(max(0,-dy),n-max(0,dy)); yt=slice(max(0,dy),n-max(0,-dy))
            a=indices[xs,ys]; b=indices[xt,yt]
            ok=valid[xs,ys]&valid[xt,yt]
            if dx and dy: ok &= valid[xt,ys]&valid[xs,yt]
            a,b=a[ok],b[ok]
            if not len(a): continue
            # <= 2.5 cm samples and margin account for the unsampled chords.
            fraction=np.linspace(0.,1.,7)
            samples=xy[a,None,:]+fraction[None,:,None]*(xy[b]-xy[a])[:,None,:]
            good=clear_segments(mapping,samples).all(axis=1)
            good &= (tree.query(samples.reshape(-1,2))[0].reshape(-1,7) >= SWEEP_RADIUS+DESIRED_GAP+.014).all(axis=1)
            a,b=a[good],b[good]
            srcs.extend((a,b));dsts.extend((b,a))
            costs.extend((np.full(len(a),np.hypot(dx,dy)*res),)*2)
        # A virtual node links the exact wheel pose to nearby grid centers.
        nearby=np.flatnonzero(valid.reshape(-1)&(np.linalg.norm(xy-pose[:2],axis=1)<=res*1.6))
        for dest in nearby:
            segment=self._samples(np.vstack((pose[:2],xy[dest])))
            if (np.all(clear_segments(mapping,segment)) and
                    tree.query(segment)[0].min() >= SWEEP_RADIUS+DESIRED_GAP+.014):
                srcs.append(np.array([n*n]));dsts.append(np.array([dest]))
                costs.append(np.array([max(1e-6,np.linalg.norm(xy[dest]-pose[:2]))]))
        if not srcs: return None
        graph=coo_matrix((np.concatenate(costs),(np.concatenate(srcs),np.concatenate(dsts))),shape=(n*n+1,n*n+1)).tocsr()
        distances,predecessor=dijkstra(graph,indices=n*n,return_predecessors=True)
        reachable=np.flatnonzero(np.isfinite(distances[:-1]))
        if not len(reachable): return None
        # Evaluate rectangular clearance while facing the observed object.
        yaw=np.arctan2(center[1]-xy[reachable,1],center[0]-xy[reachable,0])
        gaps=[]
        for offset in range(0,len(reachable),128):
            points=xy[reachable[offset:offset+128]]; angles=yaw[offset:offset+128]
            delta=surface[None,:,:]-points[:,None,:]
            x=delta[:,:,0]*np.cos(angles[:,None])+delta[:,:,1]*np.sin(angles[:,None])
            y=-delta[:,:,0]*np.sin(angles[:,None])+delta[:,:,1]*np.cos(angles[:,None])
            gaps.append(np.min(np.hypot(np.maximum(np.abs(x)-HALF_LENGTH,0.),
                                       np.maximum(np.abs(y)-HALF_WIDTH,0.)),axis=1))
        gaps=np.concatenate(gaps)
        acceptable=gaps>=DESIRED_GAP
        reachable,yaw,gaps=reachable[acceptable],yaw[acceptable],gaps[acceptable]
        if not len(reachable): return None
        # A 5 cm endpoint band absorbs grid quantization; then shortest path wins.
        minimum=float(gaps.min())
        choices=np.flatnonzero(gaps<=max(DESIRED_GAP+.05,minimum+.025))
        choice=int(choices[np.argmin(distances[reachable[choices]])])
        dest=int(reachable[choice]); route=[]; current=dest
        while current != n*n:
            if current < 0 or len(route)>n*n: return None
            route.append(xy[current]);current=int(predecessor[current])
        path=np.vstack((pose[:2],list(reversed(route))))
        endpoint=xy[dest]
        if old is None or np.linalg.norm(old['point']-endpoint)>.12:
            self.generation+=1; self.stable_since=now
        self.plan=dict(point=endpoint.copy(),path=path,yaw=float(yaw[choice]),
            goal_clearance_m=float(gaps[choice]),fallback=bool(gaps[choice]>DESIRED_GAP+.05),
            reachable_cells=len(reachable),generation=self.generation)
        return self._diagnostic(pose,surface,now)

    @staticmethod
    def _samples(path):
        parts=[np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.025))+1)) for a,b in zip(path[:-1],path[1:])]
        return np.vstack(parts) if parts else path

    def _diagnostic(self,pose,surface,now):
        p=self.plan
        return dict(p, current_clearance_m=surface_gap(pose,surface),
            endpoint_distance_m=float(np.linalg.norm(p['point']-pose[:2])),
            stable_s=now-self.stable_since,
            reason='nearest_reachable_local_position' if p['fallback'] else 'object_clearance_goal',
            scope='current_rolling_map_connected_known_free',
            footprint='0.48m square; conservative circular sweeps',
            requested_clearance_m=DESIRED_GAP)
