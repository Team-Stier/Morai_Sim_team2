"""Static course speed zones shared by RDDF exports, displays and planning."""
from pathlib import Path

import yaml


def load_course_speed_policy():
    root = Path(__file__).resolve().parents[3] / 'ros_architecture_pkg'
    if not (root / 'config/map/course_speed_policy.yaml').is_file():
        import rospkg
        root = Path(rospkg.RosPack().get_path('ros_architecture_pkg'))
    return yaml.safe_load((root / 'config/map/course_speed_policy.yaml').read_text())


class CourseSpeedZones:
    def __init__(self, route, policy):
        self.route = route
        self.policy = policy
        high = policy['high_speed']
        self.start = self.nearest_index(high['start_map_xy'])
        self.end = self.nearest_index(high['end_map_xy'])

    def nearest_index(self, point):
        return min(range(len(self.route)), key=lambda i:
                   (self.route[i][0]-point[0])**2 + (self.route[i][1]-point[1])**2)

    def unlimited(self, index):
        return (index-self.start) % len(self.route) < (self.end-self.start) % len(self.route)

    def limit_kph(self, index):
        return None if self.unlimited(index) else self.policy['normal_limit_kph']

    def split_line(self, points, indices=None):
        """Label each displayed edge; retain adjoining endpoints without joining disjoint runs."""
        if indices is None:
            indices = [self.nearest_index(p) for p in points]
        runs = []
        for i in range(len(points)-1):
            limit = self.limit_kph(indices[i])
            if not runs or runs[-1]['speed_limit_kph'] != limit:
                runs.append({'speed_limit_kph': limit, 'p': [points[i]]})
            runs[-1]['p'].append(points[i+1])
        return runs

    def annotate_lanes(self, result):
        result['course_speed_policy'] = self.policy
        for lane in result['lanes']:
            indices = [self.nearest_index(p) for p in lane['points']]
            lane['speed_limit_kph'] = [self.limit_kph(i) for i in indices]
            lane['speed_sections'] = self.split_line(lane['points'], indices)
        return result
