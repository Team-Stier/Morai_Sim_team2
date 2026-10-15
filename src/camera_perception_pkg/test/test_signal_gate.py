from pathlib import Path
from types import SimpleNamespace
import hashlib
import json
import math
import unittest
from unittest.mock import patch

import yaml
from camera_perception_pkg.signal_gate import SignalRouteGate

ROOT = Path(__file__).resolve().parents[3]
POLICY = ROOT / 'src/ros_architecture_pkg/config/map/signal_observation_policy.yaml'


class Stamp(float):
    def to_sec(self):
        return float(self)


class SignalGateTest(unittest.TestCase):
    def setUp(self):
        self.policy = yaml.safe_load(POLICY.read_text())
        self.gate = SignalRouteGate(self.policy)
        self.stop = self.policy['targets'][0]['stop_station_m']

    def context(self, progress, stamp=10., **changes):
        values = dict(header=SimpleNamespace(stamp=Stamp(stamp), frame_id='map'),
                      map_id='test-map', reference_sha256=self.policy['reference_sha256'],
                      progress=progress, route_complete=False)
        values.update(changes)
        return SimpleNamespace(**values)

    def select(self, progress):
        self.gate.update(self.context(progress), 100.)
        return self.gate.snapshot(10., 100., 10.)[0]

    def test_each_selected_stop_only_from_thirty_metres_through_stop_line(self):
        for stop in self.policy['targets']:
            station = stop['stop_station_m']
            self.assertIsNone(self.select(station-30.001))
            for remaining in (30., 20., 0.):
                self.assertEqual(self.select(station-remaining)['number'], stop['number'])
            self.assertIsNone(self.select(station+.001))
        self.assertEqual([x['number'] for x in self.policy['targets']], [1, 2, 3, 5])
        for progress in (520.6187, 530., 550.6187):
            self.assertIsNone(self.select(progress))

    def test_missing_stale_future_wrong_route_and_invalid_progress_disable(self):
        self.assertIsNone(self.gate.select(10., 100.))
        for changes in (dict(reference_sha256='wrong'), dict(map_id=''),
                        dict(progress=float('nan')), dict(progress=float('inf')),
                        dict(progress=-1.), dict(route_complete=True), dict(stamp=0.),
                        dict(stamp=10.1)):
            args = dict(progress=self.stop-10, **{})
            args.update(changes)
            self.gate.update(self.context(**args), 100.)
            self.assertIsNone(self.gate.select(10., 100.))
        self.gate.update(self.context(self.stop-10), 100.)
        self.assertIsNone(self.gate.select(10.501, 100.))
        self.assertIsNone(self.gate.select(10., 100.501))  # frozen ROS clock
        self.assertIsNone(self.gate.select(10., 100., 9.4))
        self.assertIsNone(self.gate.select(10., 100., 10.1))

    def test_scope_revision_rejects_inflight_result_after_exit_and_reentry(self):
        self.gate.update(self.context(self.stop-5), 100.)
        _, before = self.gate.snapshot(10., 100.)
        self.gate.update(self.context(self.stop+.1, stamp=10.1), 100.1)
        self.assertIsNone(self.gate.snapshot(10.1, 100.1)[0])
        self.gate.update(self.context(self.stop-5, stamp=10.2), 100.2)
        _, after = self.gate.snapshot(10.2, 100.2)
        self.assertNotEqual(before, after)
        self.gate.update(self.context(self.stop-5, stamp=9.), 100.3)
        target, reset = self.gate.snapshot(9., 100.3)
        self.assertIsNone(target)
        self.assertNotEqual(after, reset)

    def test_explicit_preview_bypass(self):
        gate = SignalRouteGate(self.policy, enabled=False)
        self.assertEqual(gate.select(0., 0.)['signal_ids'], [])

    def test_worker_preserves_camera_stamp_and_drops_result_if_route_exits_during_inference(self):
        import runpy
        import threading
        import cv2
        import numpy as np
        import rospy
        from sensor_msgs.msg import CompressedImage
        node_type = runpy.run_path(str(ROOT/'src/camera_perception_pkg/scripts/camera_perception_node'))['SignalNode']
        frame = np.zeros((16, 16, 3), dtype=np.uint8)
        message = CompressedImage(data=cv2.imencode('.jpg', frame)[1].tobytes())
        message.header.stamp = rospy.Time(10)
        message.header.frame_id = 'camera_signal_optical_frame'
        for exit_during_inference in (False, True):
            node = node_type.__new__(node_type)
            node.gate = SignalRouteGate(self.policy)
            node.hsv_config = yaml.safe_load((ROOT/"src/camera_perception_pkg/config/signal_hsv.yaml").read_text())
            node.observations_pub = SimpleNamespace(publish=lambda m:None)
            node.pending = None
            node.invalidated_revision = None
            node.condition = threading.Condition()
            images, detections = [], []
            node.image_pub = SimpleNamespace(publish=images.append)
            node.detections_pub = SimpleNamespace(publish=detections.append)
            def detect(_frame):
                if exit_during_inference:
                    node.receive_route(self.context(self.stop+.1))
                return frame, [dict(class_id=0, confidence=.9,xyxy=[2.,2.,14.,14.])]
            node.detector = detect
            with patch.object(rospy.Time, 'now', return_value=rospy.Time(10)), \
                    patch('time.monotonic', return_value=100.), \
                    patch.object(rospy, 'is_shutdown', side_effect=[False, True]), \
                    patch.object(rospy, 'loginfo_throttle'):
                node.receive_route(self.context(self.stop-10))
                node.receive(message)
                node.process()
            self.assertEqual(len(images), 0 if exit_during_inference else 1)
            if not exit_during_inference:
                self.assertEqual(images[0].header, message.header)
                output = json.loads(detections[0].data)
                self.assertTrue(output['gate_active'])
                self.assertFalse(output['driving_decision_valid'])
                self.assertEqual(output['route_scope']['signal_ids'], ['C1256W000077'])

    def test_static_policy_stations_match_original_map_stop_bar_intersections(self):
        route_path = ROOT / '참고파일들/2026_molit_comp_global_path (3).txt'
        self.assertEqual(hashlib.sha256(route_path.read_bytes()).hexdigest(), self.policy['reference_sha256'])
        route = [list(map(float, line.split())) for line in route_path.read_text().splitlines() if line.strip()]
        source = ROOT / 'src/hd_map_pkg/vendor/verdict_sdk/map-data/KATRI'
        bars = {x['idx']: x for x in json.loads((source/'lane_boundary_set.json').read_text())}
        origin = json.loads((source/'global_info.json').read_text())['local_origin_in_global']
        projection = yaml.safe_load((ROOT/'src/ros_architecture_pkg/config/tf/map_projection.yaml').read_text())
        offset = [a-b for a, b in zip(origin, projection['origin_utm_m'])]
        def cross(a, b):
            return a[0]*b[1]-a[1]*b[0]
        for target in self.policy['targets']:
            points = [[a+b for a, b in zip(p, offset)] for p in bars[target['stop_line_id']]['points']]
            station, hits = 0., []
            for a, b in zip(route, route[1:]):
                v = [b[i]-a[i] for i in (0, 1)]
                length = math.hypot(*v)
                for c, d in zip(points, points[1:]):
                    w = [d[i]-c[i] for i in (0, 1)]
                    q = [c[i]-a[i] for i in (0, 1)]
                    den = cross(v, w)
                    if abs(den) < 1e-12:
                        continue
                    t, u = cross(q, w)/den, cross(q, v)/den
                    if 0 <= t <= 1 and 0 <= u <= 1:
                        hits.append(station+t*length)
                station += length
            self.assertTrue(any(abs(hit-target['stop_station_m']) < 1e-6 for hit in hits))


if __name__ == '__main__':
    unittest.main()
