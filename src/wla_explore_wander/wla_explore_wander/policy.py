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
