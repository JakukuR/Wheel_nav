"""Pure mission invariants and graph-relative home anchor (radians, map metres)."""
from dataclasses import dataclass
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], wrap(a[2]+b[2])


def relative(a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    return c*dx+s*dy, -s*dx+c*dy, wrap(b[2]-a[2])


@dataclass(frozen=True)
class HomeAnchor:
    node_id: int
    offset: tuple

    @classmethod
    def capture(cls, home, graph):
        if not graph or not all(math.isfinite(v) for v in home) or any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in graph.values()):
            raise ValueError('home graph unavailable')
        node_id = min((i for i in graph if i > 0),
                      key=lambda i: math.hypot(graph[i][0]-home[0], graph[i][1]-home[1]))
        return cls(node_id, relative(graph[node_id], home))

    def resolve(self, graph):
        if self.node_id not in graph or not all(math.isfinite(v) for v in graph[self.node_id]):
            raise ValueError('home graph node missing')
        return compose(graph[self.node_id], self.offset)


def arrived(pose, home, xy, yaw):
    return math.hypot(pose[0]-home[0], pose[1]-home[1]) <= xy and abs(wrap(pose[2]-home[2])) <= yaw


class MissionResult:
    def __init__(self):
        self.phase = 'PREPARING'
        self.exploration = 'not_started'
        self.return_home = 'not_started'
        self.save = 'not_started'
        self.reason = ''

    @property
    def success(self):
        return self.exploration == 'complete' and self.return_home == 'succeeded' and self.save == 'succeeded'

    def snapshot(self):
        return dict(phase=self.phase, exploration_outcome=self.exploration,
                    return_outcome=self.return_home, save_outcome=self.save,
                    success=self.success, reason=self.reason)


def significant_frontiers(grid, min_length=.4):
    """Conservative raw-grid completion veto, independent of inflated-map blacklist."""
    width, height = grid.info.width, grid.info.height
    cells = grid.data
    if len(cells) != width*height or width*height > 1_000_000 or grid.info.resolution <= 0:
        raise ValueError('invalid frontier grid')
    frontier = set()
    for i, value in enumerate(cells):
        if value != -1:
            continue
        x,y = i%width,i//width
        for nx,ny in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            if 0<=nx<width and 0<=ny<height and 0<=cells[ny*width+nx]<=25:
                frontier.add(i)
                break
    count = 0
    while frontier:
        todo = [frontier.pop()]
        size = 0
        while todo:
            i = todo.pop();size += 1
            x,y=i%width,i//width
            for dx,dy in ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)):
                nx,ny=x+dx,y+dy
                j=ny*width+nx
                if 0<=nx<width and 0<=ny<height and j in frontier:
                    frontier.remove(j);todo.append(j)
        if size*grid.info.resolution >= min_length:
            count += 1
    return count


def stopped_odometry(msg, now, linear_limit, angular_limit):
    """Fresh chassis velocity evidence; VO pose remains the navigation source."""
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    v = msg.twist.twist
    values = (v.linear.x, v.linear.y, v.angular.z, now, stamp)
    return (msg.header.frame_id == 'wheel_odom' and msg.child_frame_id == 'r680_mapping_floor'
            and all(math.isfinite(value) for value in values)
            and 0 <= now - stamp <= .5
            and math.hypot(v.linear.x, v.linear.y) < linear_limit
            and abs(v.angular.z) < angular_limit)
