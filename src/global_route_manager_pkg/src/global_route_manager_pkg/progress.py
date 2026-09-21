"""Geometry-only progress tracking; checkpoints never act as ego observations."""
import bisect
import math


def lengths(points):
    result = [0.0]
    for a, b in zip(points, points[1:]):
        result.append(result[-1]+math.hypot(b[0]-a[0], b[1]-a[1]))
    return result


def corridor_bounds(lanes, margin):
    points = [point for lane in lanes for point in lane['points']]
    return (min(point[0] for point in points)-margin, max(point[0] for point in points)+margin,
            min(point[1] for point in points)-margin, max(point[1] for point in points)+margin)


def overlaps_bounds(points, bounds):
    """Keep complete source polylines, including those that traverse the envelope."""
    return (max(point[0] for point in points) >= bounds[0]
            and min(point[0] for point in points) <= bounds[1]
            and max(point[1] for point in points) >= bounds[2]
            and min(point[1] for point in points) <= bounds[3])


def project(point, points, arc, start=0.0, end=None, heading=None):
    end = arc[-1] if end is None else end
    first = max(0, bisect.bisect_right(arc, start)-1)
    last = min(len(points)-1, bisect.bisect_left(arc, end)+1)
    best = (float('inf'), start)
    for index in range(first, last):
        a, b = points[index], points[index+1]
        dx, dy = b[0]-a[0], b[1]-a[1]
        ds = arc[index+1]-arc[index]
        fraction = max(0.0, min(1.0, ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(ds*ds)))
        s = arc[index]+fraction*ds
        if s < start or s > end:
            continue
        distance_sq = (point[0]-a[0]-fraction*dx)**2+(point[1]-a[1]-fraction*dy)**2
        if heading is not None and (dx*heading[0]+dy*heading[1])/ds < 0:
            continue
        best = min(best, (distance_sq, s))
    return best


class RouteProgress:
    def __init__(self, lanes, checkpoints, checkpoint_radius_m, config):
        self.lanes = {lane['id']: lane for lane in lanes}
        self.arcs = {key: lengths(lane['points']) for key, lane in self.lanes.items()}
        self.reference = self.lanes['global_route']['points']
        self.reference_arc = self.arcs['global_route']
        self.checkpoints = checkpoints
        self.radius = checkpoint_radius_m
        self.config = config
        self.checkpoint_s = []
        start = 0.0
        for point in checkpoints:
            _, start = project(point, self.reference, self.reference_arc, start)
            self.checkpoint_s.append(start)
        self.progress = None
        self.current_lane = 'global_route'
        self.next_checkpoint = 0
        self.start_checkpoint_index = 0
        self.passed_checkpoints = []
        self.previous_point = None
        self.missed_checkpoint = False

    def update(self, point, yaw):
        heading = (math.cos(yaw), math.sin(yaw))
        if self.progress is None:
            _, self.progress = project(point, self.reference, self.reference_arc, heading=heading)
            if self.config['initialize_from_current_position']:
                self.next_checkpoint = bisect.bisect_left(self.checkpoint_s, self.progress-self.radius)
                self.start_checkpoint_index = self.next_checkpoint
        else:
            begin = max(0.0, self.progress-self.config['matching_backward_m'])
            end = min(self.reference_arc[-1], self.progress+self.config['matching_forward_m'])
            _, progress = project(point, self.reference, self.reference_arc, begin, end, heading)
            self.progress = max(self.progress, progress)

        # Lane identity follows geometry, retaining the previous lane through the
        # middle of a lateral manoeuvre until another centreline is closer.
        candidates = []
        for key, lane in self.lanes.items():
            if key == 'global_route':
                begin = max(0.0, self.progress-self.config['matching_backward_m'])
                end = min(self.reference_arc[-1], self.progress+self.config['matching_forward_m'])
            else:
                if not lane['route_s'][0]-self.config['matching_backward_m'] <= self.progress <= lane['route_s'][-1]+self.config['matching_forward_m']:
                    continue
                begin, end = 0.0, self.arcs[key][-1]
            distance_sq, _ = project(point, lane['points'], self.arcs[key], begin, end, heading)
            candidates.append((distance_sq, key != self.current_lane, key))
        self.current_lane = min(candidates)[2]

        if self.next_checkpoint < len(self.checkpoints):
            target = self.checkpoints[self.next_checkpoint]
            # The segment between consecutive measured poses catches an actual
            # radius crossing between callbacks; progress alone never passes it.
            if self.previous_point is None or self.previous_point == point:
                distance_sq = (target[0]-point[0])**2+(target[1]-point[1])**2
            else:
                distance_sq, _ = project(target, [self.previous_point, point], lengths([self.previous_point, point]))
            if distance_sq <= self.radius*self.radius:
                self.passed_checkpoints.append(self.next_checkpoint)
                self.next_checkpoint += 1
            elif self.progress > self.checkpoint_s[self.next_checkpoint]+self.radius:
                self.missed_checkpoint = True
        self.previous_point = point
        goal_index = min(self.next_checkpoint, len(self.checkpoints)-1)
        return dict(current_lane=self.current_lane, progress=self.progress,
                    next_checkpoint=self.next_checkpoint,
                    comparison_goal=self.checkpoints[goal_index],
                    comparison_goal_s=self.checkpoint_s[goal_index],
                    route_complete=len(self.passed_checkpoints) == len(self.checkpoints))
