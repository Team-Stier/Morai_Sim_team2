from pathlib import Path
import json
import math
import unittest
import yaml
import roslaunch.config

ROOT = Path(__file__).resolve().parents[3]


class SignalContractTest(unittest.TestCase):
    def test_signal_ingress_is_distinct_and_preserves_camera3_geometry(self):
        config = yaml.safe_load((ROOT / 'src/morai_interface_pkg/config/camera_signal.yaml').read_text())
        contract = yaml.safe_load((ROOT / 'src/ros_architecture_pkg/config/interface_contract.yaml').read_text())
        topic = next(t for t in contract['topics'] if t['name'] == config['topic'])
        self.assertEqual(topic['producers'], ['morai_camera_signal'])
        self.assertEqual(topic['consumers'], ['camera_perception_node'])
        self.assertEqual(topic['frame'], config['frame_id'])
        profile = json.loads((ROOT / 'src/morai_interface_pkg/config/camera3_signal_sensor.json').read_text())
        camera = next(c for c in profile['cameraList'] if c['m_SensorUniqueID'] == 3)
        self.assertEqual([float(camera['pos'][k]) for k in ('x', 'y', 'z')], [1.8, 0., 1.3])
        self.assertEqual(float(camera['rot']['pitch']), 340.)
        self.assertEqual(camera['cc']['udpConfig']['destinationPort'], config['port'])
        self.assertEqual(camera['cc']['cameraType'], 0)
        self.assertFalse(camera['cc']['vizBoundingBox2D'])
        self.assertFalse(camera['cc']['vizBoundingBox3D'])
        tf = yaml.safe_load((ROOT / 'src/ros_architecture_pkg/config/tf/frame_contract.yaml').read_text())
        self.assertTrue(all(not t['publish_enabled'] for t in tf['transforms']
                            if t['child'].startswith('camera_signal')))

    def test_world_projection_matches_source_camera_geometry(self):
        policy = yaml.safe_load((ROOT / 'src/ros_architecture_pkg/config/map/signal_observation_policy.yaml').read_text())
        extrinsics = yaml.safe_load((ROOT / 'src/ros_architecture_pkg/config/tf/sensor_extrinsics.yaml').read_text())
        mount = next(m for m in extrinsics['sensor_mounts'] if m['key'] == 'camera_signal')
        profile = json.loads((ROOT / 'src/morai_interface_pkg/config/camera3_signal_sensor.json').read_text())
        camera = next(c for c in profile['cameraList'] if c['m_SensorUniqueID'] == 3)
        projection = policy['camera_projection']
        source_xyz = [float(camera['pos'][k]) for k in ('x', 'y', 'z')]
        self.assertEqual(projection['translation_m'], source_xyz)
        self.assertEqual(mount['source_pose']['translation_m'], source_xyz)
        source_pitch = math.radians((float(camera['rot']['pitch']) + 180.) % 360. - 180.)
        self.assertAlmostEqual(projection['pitch_rad'], source_pitch)
        self.assertAlmostEqual(mount['candidate_ros_pose']['rotation_rpy_rad'][1], source_pitch)
        self.assertEqual(projection['horizontal_fov_deg'], camera['cc']['cameraFOV'])
        self.assertFalse(projection['publish_tf'])
        self.assertFalse(mount['publish_enabled'])

    def test_main_stack_starts_single_camera3_and_can_disable_it(self):
        launch = str(ROOT / 'src/system_bringup_pkg/launch/frenet_all.launch')
        config = roslaunch.config.load_config_default([(launch, ['send_to_morai:=false', 'rviz:=false'])], None)
        names = [n.name for n in config.nodes]
        self.assertEqual(names.count('morai_camera_signal'), 1)
        self.assertEqual(names.count('camera_perception_node'), 1)
        self.assertTrue(config.params['/camera_perception_node/enable_signal'].value)
        self.assertTrue(config.params['/camera_perception_node/enable_semantic'].value)
        self.assertTrue(config.params['/camera_perception_node/signal_route_gate'].value)
        policy = config.params['/camera_perception_node/signal_policy/targets'].value
        self.assertEqual([t['number'] for t in policy], [1, 2, 3, 5])
        self.assertEqual(config.params['/morai_camera_signal/port'].value, 9297)
        disabled = roslaunch.config.load_config_default([(launch, ['camera3_yolo:=false'])], None)
        self.assertNotIn('morai_camera_signal', [n.name for n in disabled.nodes])

    def test_pointpainting_launch_remains_pidnet_only(self):
        launch = str(ROOT / 'src/system_bringup_pkg/launch/pointpainting.launch')
        config = roslaunch.config.load_config_default([(launch, ['weights:=/tmp/test.pt'])], None)
        self.assertTrue(config.params['/camera_perception_node/enable_semantic'].value)
        self.assertFalse(config.params['/camera_perception_node/enable_signal'].value)

    def test_route_gate_uses_approved_producer_and_explicit_preview_override(self):
        contract = yaml.safe_load((ROOT/'src/ros_architecture_pkg/config/interface_contract.yaml').read_text())
        route = next(t for t in contract['topics'] if t['name'] == '/molit/route/context')
        self.assertEqual(route['producers'], ['global_route_manager_node'])
        self.assertIn('camera_perception_node', route['consumers'])
        self.assertIn(route['name'], contract['package_boundaries']['camera_perception_pkg']['inputs'])
        launch = str(ROOT/'src/system_bringup_pkg/launch/camera3_signal.launch')
        normal = roslaunch.config.load_config_default([(launch, [])], None)
        preview = roslaunch.config.load_config_default([(launch, ['signal_route_gate:=false'])], None)
        self.assertTrue(normal.params['/camera_perception_node/signal_route_gate'].value)
        self.assertFalse(preview.params['/camera_perception_node/signal_route_gate'].value)
