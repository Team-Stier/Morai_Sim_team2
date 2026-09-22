"""Frenet candidates and ETA selection; geometry is measured cluster points only."""
from dataclasses import dataclass, field
import math
import numpy as np


def arc(xy):
    return np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]


def sample(xy, s, q):
    return np.column_stack([np.interp(q, s, xy[:, i]) for i in range(xy.shape[1])])


def project(xy, s, p):
    v = np.diff(xy[:, :2], axis=0)
    u = np.clip(np.sum((p-xy[:-1, :2])*v, axis=1)/np.sum(v*v, axis=1), 0, 1)
    residual = p-xy[:-1, :2]-u[:, None]*v
    i = int(np.argmin(np.sum(residual*residual, axis=1)))
    tangent = v[i]/np.linalg.norm(v[i])
    return s[i]+u[i]*(s[i+1]-s[i]), float(np.cross(tangent, residual[i])), math.atan2(tangent[1], tangent[0])


def smooth(u):
    u = np.clip(u, 0, 1)
    return 10*u**3-15*u**4+6*u**5


def quintic(start, slope, end, distance, q):
    """d, d/ds and d²/ds² boundary conditions; slope is not a time derivative."""
    a = np.array([[distance**3, distance**4, distance**5],
                  [3*distance**2, 4*distance**3, 5*distance**4],
                  [6*distance, 12*distance**2, 20*distance**3]])
    b = np.array([end-start-slope*distance, -slope, 0.])
    c = np.linalg.solve(a, b)
    u = np.clip(q, 0, distance)
    return start+slope*u+c[0]*u**3+c[1]*u**4+c[2]*u**5


@dataclass
class Lane:
    id: str
    xy: np.ndarray
    s: np.ndarray
    limits: np.ndarray
    successors: list = field(default_factory=list)


@dataclass
class Window:
    source: str
    target: str
    start: float
    end: float


@dataclass
class Obstacle:
    points: np.ndarray
    velocity: np.ndarray
    velocity_valid: bool = True
    age: float = 0.


@dataclass
class Candidate:
    key: str
    target: str
    xy: np.ndarray
    route_s: np.ndarray
    limits: np.ndarray
    changes: int = 0
    change_end: float = 0.
    return_start: float = math.inf
    speed: np.ndarray = field(default_factory=lambda: np.array([]))
    times: np.ndarray = field(default_factory=lambda: np.array([]))
    cost: float = math.inf
    eta: float = math.inf
    comfort: float = 0.
    reason: str = 'unevaluated'
    feasible: bool = False
    wait: float = 0.
    geometry_cache: tuple = None


def geometry(xy):
    s = arc(xy)
    v = np.gradient(xy[:, :2], s, axis=0)
    theta = np.unwrap(np.arctan2(v[:, 1], v[:, 0]))
    return s, theta, np.gradient(theta, s)


def candidate_geometry(candidate):
    if candidate.geometry_cache is None:
        candidate.geometry_cache = geometry(candidate.xy)
    return candidate.geometry_cache


class ObstacleGrid:
    """Index objects by their measured-point swept XY cells; points stay unchanged."""
    def __init__(self, objects, config):
        self.objects = objects
        self.cell = config['obstacle_grid_cell_m']
        self.radius = math.hypot(config['front_overhang_m'],config['vehicle_width_m']/2)
        self.cells = {}
        horizon = config['prediction_horizon_sec']
        for index, obj in enumerate(objects):
            start = obj.points[:,:2]+(obj.age*obj.velocity[:2] if obj.velocity_valid else 0.)
            end = start+(horizon*obj.velocity[:2] if obj.velocity_valid else 0.)
            low = np.floor((np.minimum(start.min(axis=0),end.min(axis=0))-self.radius)/self.cell).astype(int)
            high = np.floor((np.maximum(start.max(axis=0),end.max(axis=0))+self.radius)/self.cell).astype(int)
            for x in range(low[0],high[0]+1):
                for y in range(low[1],high[1]+1):
                    self.cells.setdefault((x,y),[]).append(index)

    def near(self, position):
        key = tuple(np.floor(position[:2]/self.cell).astype(int))
        return [self.objects[i] for i in self.cells.get(key,())]


def geometry_windows(lanes, config):
    """Adjacent, same-direction RDDF overlaps; no map markings or topology."""
    windows = []
    keys = list(lanes)
    step = config['spatial_step_m']
    for i, source_id in enumerate(keys):
        source = lanes[source_id]
        for target_id in keys[i+1:]:
            target = lanes[target_id]
            begin, end = max(source.s[0],target.s[0]), min(source.s[-1],target.s[-1])
            if end-begin < 2*step:
                continue
            qs = np.arange(begin,end,step)
            a, b = sample(source.xy,source.s,qs), sample(target.xy,target.s,qs)
            ta, tb = np.gradient(a[:,:2],axis=0), np.gradient(b[:,:2],axis=0)
            ta /= np.linalg.norm(ta,axis=1)[:,None]
            tb /= np.linalg.norm(tb,axis=1)[:,None]
            delta = b[:,:2]-a[:,:2]
            distance = np.linalg.norm(delta,axis=1)
            along = np.abs(np.sum(delta*ta,axis=1))
            valid = ((distance >= config['rddf_neighbor_min_m']) &
                     (distance <= config['rddf_neighbor_max_m']) &
                     (along <= config['rddf_neighbor_min_m']) &
                     (np.sum(ta*tb,axis=1) >= config['rddf_heading_dot_min']) &
                     (np.abs(a[:,2]-b[:,2]) <= config['rddf_height_difference_m']))
            transitions = np.diff(np.r_[False,valid,False].astype(int))
            for first, last in zip(np.flatnonzero(transitions==1),np.flatnonzero(transitions==-1)):
                if last-first < 2:
                    continue
                windows.extend([Window(source_id,target_id,float(qs[first]),float(qs[last-1])),
                                Window(target_id,source_id,float(qs[first]),float(qs[last-1]))])
    return windows


def footprint_hit(points, position, theta, c):
    """No hull or obstacle box: test actual returns against ego footprint."""
    delta = points[:, :2]-position[:2]
    x = math.cos(theta)*delta[:, 0]+math.sin(theta)*delta[:, 1]
    y = -math.sin(theta)*delta[:, 0]+math.cos(theta)*delta[:, 1]
    margin = c['object_margin_m']
    return np.any((x >= -c['rear_overhang_m']-margin) &
                  (x <= c['front_overhang_m']+margin) &
                  (np.abs(y) <= c['vehicle_width_m']/2+margin))



def footprint_hits(points, positions, headings, c, point_shifts=None):
    """Batch the same point/footprint test, bounding temporary array sizes."""
    result = np.zeros(len(positions), dtype=bool)
    margin = c['object_margin_m']
    for first in range(0, len(positions), 32):
        poses = positions[first:first+32, :2]
        angles = headings[first:first+32]
        co, si = np.cos(angles)[:, None], np.sin(angles)[:, None]
        hit = np.zeros(len(poses), dtype=bool)
        for point_first in range(0, len(points), 512):
            sample_points = points[None, point_first:point_first+512, :2]
            if point_shifts is not None:
                sample_points = sample_points+point_shifts[first:first+len(poses), None, :]
            delta = sample_points-poses[:, None, :]
            x = co*delta[:, :, 0]+si*delta[:, :, 1]
            y = -si*delta[:, :, 0]+co*delta[:, :, 1]
            hit |= np.any((x >= -c['rear_overhang_m']-margin) &
                          (x <= c['front_overhang_m']+margin) &
                          (np.abs(y) <= c['vehicle_width_m']/2+margin), axis=1)
            if hit.all():
                break
        result[first:first+len(poses)] = hit
    return result


def boundary_hit(position, theta, lines, c):
    """Segment/rectangle clipping catches wheel contact and whole-body crossing."""
    segments = np.asarray(lines).reshape(-1, 2, 3)
    if len(segments) == 0:
        return False
    delta = segments[:, :, :2]-position[:2]
    radius = math.hypot(c['front_overhang_m'],c['vehicle_width_m']/2)
    near = np.all(delta.min(axis=1) <= radius,axis=1)&np.all(delta.max(axis=1) >= -radius,axis=1)
    delta = delta[near]
    co, si = math.cos(theta), math.sin(theta)
    local = np.stack((co*delta[:,:,0]+si*delta[:,:,1],-si*delta[:,:,0]+co*delta[:,:,1]),axis=2)
    a, v = local[:,0], local[:,1]-local[:,0]
    lo, hi = np.zeros(len(a)), np.ones(len(a))
    for j, lower, upper in ((0,-c['rear_overhang_m'],c['front_overhang_m']),
                            (1,-c['vehicle_width_m']/2,c['vehicle_width_m']/2)):
        parallel = np.abs(v[:,j]) < 1e-12
        hi[parallel & ((a[:,j]<lower)|(a[:,j]>upper))] = -1
        t0 = np.divide(lower-a[:,j],v[:,j],out=np.full(len(a),-np.inf),where=~parallel)
        t1 = np.divide(upper-a[:,j],v[:,j],out=np.full(len(a),np.inf),where=~parallel)
        lo,hi = np.maximum(lo,np.minimum(t0,t1)),np.minimum(hi,np.maximum(t0,t1))
    return bool(np.any(lo<=hi))


class Planner:
    def __init__(self, config):
        self.c = config
        self.pending = None
        self.pending_since = 0.
        self.committed = None
        self.last_candidates = []
        self.boundary_source = None
        self.boundary_tree = None

    def boundary_stop(self, candidate, s, theta, boundaries):
        if self.boundary_source is not boundaries:
            self.boundary_source = boundaries
            self.boundary_segments = np.asarray([pair for line in boundaries for pair in zip(line[:-1],line[1:])]).reshape(-1,2,3)
            if len(self.boundary_segments):
                self.boundary_tree = {}
                low=np.floor(self.boundary_segments[:,:,:2].min(axis=1)/10).astype(int)
                high=np.floor(self.boundary_segments[:,:,:2].max(axis=1)/10).astype(int)
                for i,(a,b) in enumerate(zip(low,high)):
                    for x in range(a[0],b[0]+1):
                        for y in range(a[1],b[1]+1):
                            self.boundary_tree.setdefault((x,y),[]).append(i)
            else:
                self.boundary_tree = None
        if self.boundary_tree is None:
            return math.inf
        qs = np.arange(0,s[-1]+1e-6,self.c['collision_step_m'])
        positions = sample(candidate.xy,s,qs)
        headings = np.interp(qs,s,theta)
        radius = math.hypot(self.c['front_overhang_m'],self.c['vehicle_width_m']/2)
        low=np.floor((positions[:,:2]-radius)/10).astype(int)
        high=np.floor((positions[:,:2]+radius)/10).astype(int)
        hits=[list({i for x in range(a[0],b[0]+1) for y in range(a[1],b[1]+1)
                    for i in self.boundary_tree.get((x,y),[])}) for a,b in zip(low,high)]
        pose_ids = np.repeat(np.arange(len(positions)),[len(x) for x in hits])
        if len(pose_ids)==0:
            return math.inf
        segment_ids = np.concatenate([x for x in hits if x]).astype(int)
        delta = self.boundary_segments[segment_ids]-positions[pose_ids,None,:]
        height_ok = (delta[:,:,2].min(axis=1)<=self.c['map_height_tolerance_m']) & (delta[:,:,2].max(axis=1)>=-self.c['map_height_tolerance_m'])
        co,si = np.cos(headings[pose_ids])[:,None],np.sin(headings[pose_ids])[:,None]
        local = np.stack((co*delta[:,:,0]+si*delta[:,:,1],-si*delta[:,:,0]+co*delta[:,:,1]),axis=2)
        a,v = local[:,0],local[:,1]-local[:,0]
        lo,hi=np.zeros(len(a)),np.ones(len(a))
        for j,lower,upper in ((0,-self.c['rear_overhang_m'],self.c['front_overhang_m']),(1,-self.c['vehicle_width_m']/2,self.c['vehicle_width_m']/2)):
            parallel=np.abs(v[:,j])<1e-12
            hi[parallel&((a[:,j]<lower)|(a[:,j]>upper))]=-1
            t0=np.divide(lower-a[:,j],v[:,j],out=np.full(len(a),-np.inf),where=~parallel)
            t1=np.divide(upper-a[:,j],v[:,j],out=np.full(len(a),np.inf),where=~parallel)
            lo,hi=np.maximum(lo,np.minimum(t0,t1)),np.minimum(hi,np.maximum(t0,t1))
        collisions=pose_ids[(lo<=hi)&height_ok]
        return float(qs[collisions.min()]) if len(collisions) else math.inf

    def chain(self, key, lanes, end):
        parts, seen = [], set()
        while key in lanes and key not in seen:
            lane = lanes[key]
            seen.add(key)
            mask = np.ones(len(lane.s), dtype=bool) if not parts else lane.s > parts[-1].s[-1]+1e-6
            if np.any(mask):
                parts.append(Lane(key, lane.xy[mask], lane.s[mask], lane.limits[mask]))
            if lane.s[-1] >= end:
                break
            next_ids = [x for x in lane.successors if x in lanes and x not in seen]
            if not next_ids:
                break
            key = min(next_ids, key=lambda x: abs(lanes[x].s[0]-lane.s[-1]))
        return Lane(parts[0].id, np.vstack([p.xy for p in parts]),
                    np.concatenate([p.s for p in parts]), np.concatenate([p.limits for p in parts]))

    def candidates(self, lanes, windows, current, progress, goal_s, ego, heading, speed):
        c = self.c
        reference = lanes['global_route']
        # Route and EgoState arrive independently. Anchor geometry to this
        # measured ego pose, otherwise the second sample can lie behind it.
        progress = project(reference.xy, reference.s, ego[:2])[0]
        if c.get('loop_route', False):
            length = reference.s[-1]
            extended = dict(lanes)
            extended['global_route'] = Lane('global_route',
                np.vstack((reference.xy, reference.xy[1:])),
                np.r_[reference.s, reference.s[1:]+length],
                np.r_[reference.limits, reference.limits[1:]], reference.successors)
            lanes = extended
            reference = lanes['global_route']
        end = min(reference.s[-1], max(goal_s+c['tail_m'], progress+c['minimum_path_m']))
        qs = np.arange(progress, end, c['spatial_step_m'])
        ref = sample(reference.xy, reference.s, qs)
        tangent = np.gradient(ref[:, :2], qs, axis=0)
        tangent /= np.linalg.norm(tangent, axis=1)[:, None]
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        d0 = np.dot(ego[:2]-ref[0, :2], normal[0])
        ref_heading = math.atan2(tangent[0, 1], tangent[0, 0])
        initial_slope = math.tan(np.clip(math.atan2(math.sin(heading-ref_heading), math.cos(heading-ref_heading)), -1., 1.))
        source = self.chain(current, lanes, end)
        source_xy = sample(source.xy, source.s, qs)
        source_d = np.sum((source_xy[:, :2]-ref[:, :2])*normal, axis=1)
        source_limits = np.interp(qs, source.s, source.limits)
        results = []

        def make(key, target, d, limits, changes=0, change_end=0., return_start=math.inf):
            # Match current measured d and heading; zero lateral jump at t=0.
            distance = max(c['connection_m'], speed*c['connection_sec'])
            slope = (d[1]-d[0])/(qs[1]-qs[0])
            correction = quintic(d0-d[0], initial_slope-slope, 0., distance, qs-progress)
            xyz = ref.copy()
            xyz[:, :2] += (d+correction)[:, None]*normal
            xyz[0, :2] = ego[:2]
            limits = limits.copy()
            if end == reference.s[-1]:
                limits[-1] = 0.
            results.append(Candidate(key, target, xyz, qs.copy(), limits, changes, change_end, return_start))

        nominal = max(speed, c['lane_change_speed_mps'])
        if c['rddf_geometry_only'] and current != 'global_route' and source.s[-1] < end:
            # np.interp holds a short RDDF's endpoint beyond its domain. Its
            # projection on a turning reference is not a continuation lane.
            # Finish the return while the source RDDF still exists.
            return_end = source.s[-1]-c['front_overhang_m']
            return_start = return_end-max(c['connection_m'], nominal*max(c['change_times_sec']))
            return_weight = smooth((qs-return_start)/(return_end-return_start))
            keep_d = source_d*(1-return_weight)
            reference_limits = np.interp(qs, reference.s, reference.limits)
            keep_limits = np.where(qs >= return_end, reference_limits, source_limits)
            make('keep', 'global_route', keep_d, keep_limits, 1, return_end, return_start)
        else:
            make('keep', current, source_d, source_limits)
        for window in windows:
            if window.source != current or window.target not in lanes:
                continue
            target = self.chain(window.target, lanes, end)
            target_xy = sample(target.xy, target.s, qs)
            target_d = np.sum((target_xy[:, :2]-ref[:, :2])*normal, axis=1)
            for prepare in c['prepare_times_sec']:
                for duration in c['change_times_sec']:
                    start = max(progress+prepare*nominal, window.start+c['rear_overhang_m'])
                    stop = start+nominal*duration
                    if stop+c['front_overhang_m'] > window.end or stop >= goal_s:
                        continue
                    weight = smooth((qs-start)/(stop-start))
                    d = source_d+(target_d-source_d)*weight
                    limits = np.minimum(source_limits, np.interp(qs, target.s, target.limits))
                    source_cap = np.where(source_limits < 0, c['high_cruise_kph']/3.6, source_limits)
                    target_limits = np.interp(qs, target.s, target.limits)
                    target_cap = np.where(target_limits < 0,c['high_cruise_kph']/3.6,target_limits)
                    limits = np.minimum(source_cap,target_cap)
                    changes, return_start = 1, math.inf
                    # Same checkpoint: a different lane must legally return when
                    # it does not meet the reference goal or ends before it.
                    at_goal = int(np.searchsorted(qs, goal_s).clip(0, len(qs)-1))
                    if ((not c['rddf_geometry_only'] and abs(d[at_goal]) > c['checkpoint_radius_m'])
                            or target.s[-1] < goal_s):
                        backs = [w for w in windows if w.source in (window.target, target.id) and w.target == 'global_route']
                        backs = [w for w in backs if min(w.end-c['front_overhang_m'], goal_s)-nominal*duration >= max(w.start+c['rear_overhang_m'], stop)]
                        if not backs:
                            continue
                        back = max(backs, key=lambda w:w.end)
                        back_end = min(back.end-c['front_overhang_m'], goal_s)
                        return_start = back_end-nominal*duration
                        d *= 1-smooth((qs-return_start)/(back_end-return_start))
                        changes = 2
                    if changes == 1 and target.s[-1] < end:
                        limits[qs > target.s[-1]] = 0.
                    make('%s:%.1f:%.1f'%(window.target, prepare, duration), window.target, d, limits, changes, back_end if changes==2 else stop, return_start)
        return results

    def obstacle_detours(self, reference, objects):
        """Return to the current RDDF after a nearby static measured obstruction."""
        c = self.c
        if not c.get('local_detour_enabled', False):
            return []
        s, theta, _ = candidate_geometry(reference)
        hits = []
        for obj in objects:
            if obj.velocity_valid and np.linalg.norm(obj.velocity[:2]) > c['stationary_speed_mps']:
                continue
            points = obj.points.copy()
            if obj.velocity_valid:
                points[:, :2] += obj.age*obj.velocity[:2]
            indices = np.flatnonzero(s <= c['collision_precision_distance_m'])
            hits.extend(indices[footprint_hits(points, reference.xy[indices], theta[indices], c)])
        if not hits:
            return []
        normal = np.column_stack((-np.sin(theta), np.cos(theta)))
        results = []
        for length in c['local_detour_transition_m']:
            return_begin = max(length, s[max(hits)]+c['local_detour_clearance_m'])
            finish = return_begin+length
            if finish >= min(s[-1], c['collision_precision_distance_m']):
                continue
            weight = smooth(s/length)*(1-smooth((s-return_begin)/length))
            for offset in c['local_detour_offsets_m']:
                xyz = reference.xy.copy()
                xyz[:, :2] += (offset*weight)[:, None]*normal
                limits = reference.limits.copy()
                maneuver = s <= finish
                limits[maneuver] = np.minimum(
                    np.where(limits[maneuver] < 0, math.inf, limits[maneuver]),
                    c['local_detour_speed_kph']/3.6+c['normal_cruise_margin_kph']/3.6)
                results.append(Candidate('detour:%.1f:%.1f' % (offset, length), reference.target,
                    xyz, reference.route_s.copy(), limits, 2,
                    float(np.interp(finish, s, reference.route_s))))
        return results

    def profile(self, candidate, speed, stop=math.inf, cap=None):
        c = self.c
        s, theta, curvature = candidate_geometry(candidate)
        limits = np.where(candidate.limits < 0, c['high_cruise_kph']/3.6,
                          np.maximum(0, candidate.limits-c['normal_cruise_margin_kph']/3.6))
        if c['test_speed_cap_kph'] > 0:
            limits = np.minimum(limits, c['test_speed_cap_kph']/3.6)
        limits = np.minimum(limits, np.sqrt(c['lateral_acceleration_mps2']/np.maximum(np.abs(curvature), 1e-8)))
        if cap is not None:
            limits = np.minimum(limits, cap)
        limits[s >= stop] = 0.
        v = limits.copy()
        for i in range(len(v)-2, -1, -1):
            v[i] = min(v[i], math.sqrt(v[i+1]**2+2*c['deceleration_mps2']*(s[i+1]-s[i])))
        current_speed = max(speed, 0.)
        overspeed = current_speed > v[0]+c['speed_tolerance_mps']
        v[0] = current_speed
        for i in range(1, len(v)):
            v[i] = min(v[i], math.sqrt(v[i-1]**2+2*c['acceleration_mps2']*(s[i]-s[i-1])))
            if overspeed:
                # Keep driving while applying the configured deceleration until
                # the candidate's geometric speed cap becomes reachable.
                reachable_min = math.sqrt(max(0.,v[i-1]**2-2*c['deceleration_mps2']*(s[i]-s[i-1])))
                v[i] = max(v[i],reachable_min)
        times = np.zeros(len(v))
        moving = v[1:]+v[:-1] > 1e-7
        dt = np.full(len(v)-1, math.inf)
        dt[moving] = 2*np.diff(s)[moving]/(v[1:]+v[:-1])[moving]
        times[1:] = np.cumsum(dt)
        return s, theta, curvature, v, times

    def collision(self, xy, theta, times, objects, start_delay=0.):
        c = self.c
        # Both spatial and temporal interpolation: short clusters cannot fall
        # between coarse trajectory samples, including a fast rear vehicle.
        obstacle_grid = objects if isinstance(objects,ObstacleGrid) else ObstacleGrid(objects,c)
        travelled = 0.
        positions, angles, stamps, segments = [], [], [], []
        for i in range(len(xy)-1):
            travelled += np.linalg.norm(xy[i+1,:2]-xy[i,:2])
            if travelled > c['collision_precision_distance_m']:
                break
            if not np.isfinite(times[i+1]) or times[i]+start_delay > c['prediction_horizon_sec']:
                break
            duration = times[i+1]-times[i]
            inspected_duration = min(duration, c['prediction_horizon_sec']-times[i]-start_delay)
            end_fraction = inspected_duration/duration if duration > 0 else 1.
            count = max(1, int(math.ceil(end_fraction*np.linalg.norm(xy[i+1, :2]-xy[i, :2])/c['collision_step_m'])),
                        int(math.ceil(inspected_duration/c['collision_time_step_sec'])))
            u = np.linspace(0, end_fraction, count+1)
            positions.append(xy[i]*(1-u[:,None])+xy[i+1]*u[:,None])
            angles.append(theta[i]*(1-u)+theta[i+1]*u)
            stamps.append(times[i]+u*(times[i+1]-times[i])+start_delay)
            segments.extend([i]*len(u))
        if not positions:
            return None
        positions = np.concatenate(positions)
        angles, stamps = np.concatenate(angles), np.concatenate(stamps)
        segments = np.asarray(segments)
        # Reuse the same grid membership and interpolation samples as the scalar
        # search, but test each nearby object's samples in bounded batches.
        cells = np.floor(positions[:,:2]/obstacle_grid.cell).astype(int)
        groups = {}
        for index, cell in enumerate(cells):
            groups.setdefault(tuple(cell), []).append(index)
        earliest = None
        for cell, indices in groups.items():
            indices = np.asarray(indices)
            if earliest is not None:
                indices = indices[segments[indices] < earliest]
            if not len(indices):
                continue
            for object_id in obstacle_grid.cells.get(cell, ()):
                obj = obstacle_grid.objects[object_id]
                shifts = ((stamps[indices]+obj.age)[:,None]*obj.velocity[:2]
                          if obj.velocity_valid else None)
                hits = footprint_hits(obj.points, positions[indices], angles[indices], c, shifts)
                if hits.any():
                    hit = int(segments[indices[np.flatnonzero(hits)[0]]])
                    earliest = hit if earliest is None else min(earliest,hit)
                    if earliest == 0:
                        return 0
        return earliest

    def evaluate(self, candidate, speed, objects, boundaries, goal_s):
        c = self.c
        obstacle_grid = objects if isinstance(objects,ObstacleGrid) else ObstacleGrid(objects,c)
        raw_objects = obstacle_grid.objects
        result = self.profile(candidate, speed)
        s, theta, curvature, v, times = result
        reference_path = candidate.key in ('keep','committed')
        if (not reference_path and
                np.max(np.abs(np.arctan(c['wheelbase_m']*curvature))) > c['max_steering_rad']):
            candidate.reason = 'steering_limit'
            return candidate
        # All portions of a changing vehicle stay clear of forbidden boundaries.
        boundary_stop = self.boundary_stop(candidate,s,theta,boundaries)
        if candidate.changes and math.isfinite(boundary_stop):
            candidate.reason = 'forbidden_boundary'
            return candidate
        # Spatial lead projection supplies stopping/following speeds; no box
        # proxy is substituted for the measured obstacle shape.
        stop = max(0., boundary_stop-c['stop_margin_m'])
        known_static_stop=stop
        cap = np.full(len(s), math.inf)
        precise = s <= c['collision_precision_distance_m']
        for obj in raw_objects:
            predicted = obj.points.copy()
            if obj.velocity_valid:
                predicted[:, :2] += obj.age*obj.velocity[:2]
            indices = np.flatnonzero(precise)
            hits = indices[footprint_hits(predicted, candidate.xy[indices], theta[indices], c)]
            if not len(hits):
                continue
            index = hits[0]
            forward_velocity = float(np.dot(obj.velocity[:2], [math.cos(theta[index]), math.sin(theta[index])])) if obj.velocity_valid else 0.
            if forward_velocity <= c['stationary_speed_mps']:
                stop = min(stop, max(0., s[index]-c['stop_margin_m']))
                if not obj.velocity_valid or np.linalg.norm(obj.velocity[:2])<=c['stationary_speed_mps']:
                    known_static_stop=min(known_static_stop,stop)
            else:
                remaining = np.maximum(0., s[index]-s-c['follow_gap_m'])
                cap = np.minimum(cap, forward_velocity+remaining/c['follow_time_sec'])
        result = self.profile(candidate, speed, stop, cap)
        if result is None:
            candidate.reason = 'insufficient_stopping_distance'
            return candidate
        s, theta, curvature, v, times = result
        collision = self.collision(candidate.xy, theta, times, obstacle_grid)
        if collision is not None:
            if candidate.changes:
                candidate.reason = 'predicted_cluster_collision'
                return candidate
            stop = min(stop, max(0., s[collision]-c['stop_margin_m']))
            result = self.profile(candidate, speed, stop)
            if result is None:
                candidate.reason = 'insufficient_stopping_distance'
                return candidate
            s, theta, curvature, v, times = result
            if self.collision(candidate.xy, theta, times, obstacle_grid) is not None:
                candidate.reason = 'no_collision_free_stop'
                return candidate
        candidate.speed, candidate.times = v, times
        candidate.feasible = True
        goal = min(int(np.searchsorted(candidate.route_s, goal_s)), len(s)-1)
        eta = times[goal]
        finite_cost=np.flatnonzero(np.isfinite(times[:goal+1]))
        cost_times,cost_speed,cost_curvature=times[finite_cost],v[finite_cost],curvature[finite_cost]
        candidate.reason = 'following' if np.any(np.isfinite(cap)) else 'feasible'
        if not np.isfinite(eta):
            candidate.reason = ('forbidden_boundary_stop' if math.isfinite(boundary_stop)
                                else 'stop_wait_prediction_unresolved')
            # Evaluate finite waits only when measured moving clusters clear.
            moving = known_static_stop>s[goal] and any(o.velocity_valid and np.linalg.norm(o.velocity[:2]) > c['stationary_speed_mps'] for o in raw_objects)
            if moving:
                stop_index=int(np.flatnonzero(np.isfinite(times))[-1])
                rest=Candidate('resume',candidate.target,candidate.xy[stop_index:],candidate.route_s[stop_index:],candidate.limits[stop_index:])
                free = self.profile(rest, 0.)
                for delay in c['wait_times_sec']:
                    offset=times[stop_index]+delay
                    if offset>c['prediction_horizon_sec']:
                        continue
                    waiting_collision=any(footprint_hit(o.points+np.r_[o.velocity[:2]*(t+o.age),0.],candidate.xy[stop_index],theta[stop_index],c)
                                          for t in np.arange(times[stop_index],offset,c['collision_time_step_sec']) for o in raw_objects)
                    if not waiting_collision and self.collision(rest.xy, free[1], free[4], obstacle_grid, offset) is None:
                        eta = free[4][goal-stop_index]+offset
                        candidate.wait = delay
                        candidate.reason = 'waiting_for_crossing'
                        finish=goal-stop_index+1
                        cost_times=np.r_[times[:stop_index+1],offset,free[4][1:finish]+offset]
                        cost_speed=np.r_[v[:stop_index+1],0.,free[3][1:finish]]
                        cost_curvature=np.r_[curvature[:stop_index+1],curvature[stop_index],free[2][1:finish]]
                        break
        if len(cost_times) > 2:
            ts, vs = cost_times, cost_speed
            steering = np.arctan(c['wheelbase_m']*cost_curvature)
            rates = np.diff(steering)/np.diff(ts)
            if not reference_path and np.max(np.abs(rates)) > c['max_steering_rate_rps']:
                candidate.feasible = False
                candidate.reason = 'steering_rate_limit'
                return candidate
            acceleration = np.diff(vs)/np.diff(ts)
            candidate.comfort = float(np.mean((acceleration/2.)**2+(rates/1.5)**2))
        candidate.eta = float(eta)
        candidate.cost = candidate.eta+c['lane_change_penalty_sec']*candidate.changes+c['comfort_penalty_sec']*candidate.comfort
        return candidate

    def select(self, candidates, now, progress):
        self.last_candidates = candidates
        keep = candidates[0]
        feasible = [x for x in candidates if x.feasible]
        if self.committed is not None:
            retained = next((x for x in candidates if x.key == 'committed' and x.feasible), None)
            if retained is not None:
                if progress >= self.committed.change_end and math.isinf(self.committed.return_start):
                    self.committed = None
                return retained
            self.committed = None
        if not feasible:
            return None
        best = min(feasible, key=lambda x:x.cost)
        if best is keep or not math.isfinite(best.cost) or keep.cost-best.cost < self.c['minimum_gain_sec']:
            self.pending = None
            return keep if keep.feasible else None
        if self.pending != best.target:
            self.pending, self.pending_since = best.target, now
        if now-self.pending_since < self.c['gain_confirmation_sec']:
            return keep if keep.feasible else None
        self.committed, self.pending = best, None
        return best
