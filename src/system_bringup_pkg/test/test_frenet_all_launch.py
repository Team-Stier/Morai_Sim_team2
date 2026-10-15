from pathlib import Path
import unittest

import roslaunch.config


LAUNCH = str(Path(__file__).resolve().parents[1] / 'launch/frenet_all.launch')


class FrenetAllLaunchTest(unittest.TestCase):
    def load(self, args=()):
        return roslaunch.config.load_config_default([(LAUNCH, list(args))], None)

    def test_expanded_stack_has_single_producers_and_rviz(self):
        config = self.load()
        names = [node.namespace + node.name for node in config.nodes]
        self.assertEqual(len(names), len(set(names)))
        self.assertTrue(all(node.namespace == '/' for node in config.nodes))
        for name in ('morai_gps_bridge', 'morai_imu_bridge', 'morai_velodyne_cloud',
                     'localization_node', 'sensor_tf_publisher', 'vehicle_rviz',
                     'hd_map_server_node', 'global_route_manager_node', 'world_model_node',
                     'path_planner_node', 'vehicle_controller_node', 'safety_supervisor_node',
                     'morai_control_sender'):
            self.assertIn('/' + name, names)
        self.assertIn('/camera_perception_node', names)
        self.assertIn('/pointpainting_node', names)
        self.assertNotIn('/lidar_perception_node', names)
        self.assertTrue(config.params['/world_model_node/pidnet_enabled'].value)
        self.assertTrue(config.params['/pointpainting_node/foreground_only'].value)
        self.assertFalse(config.params['/use_sim_time'].value)
        self.assertEqual(config.params['/path_planner_node/test_speed_cap_kph'].value, 0.)
        self.assertEqual(config.params['/morai_control_sender/port'].value, 9093)

    def test_diagnostic_options_reach_owned_launch_files(self):
        config = self.load(['rviz:=false', 'send_to_morai:=false', 'test_speed_cap_kph:=10.0'])
        names = {node.name for node in config.nodes}
        self.assertNotIn('vehicle_rviz', names)
        self.assertNotIn('morai_control_sender', names)
        self.assertIn('vehicle_controller_node', names)
        self.assertEqual(config.params['/path_planner_node/test_speed_cap_kph'].value, 10.)

    def test_explicit_legacy_mode_does_not_start_pidnet(self):
        config = self.load(['pidnet_enabled:=false'])
        names = {node.name for node in config.nodes}
        self.assertIn('lidar_perception_node', names)
        self.assertIn('camera_perception_node', names)
        self.assertFalse(config.params['/camera_perception_node/enable_semantic'].value)
        self.assertTrue(config.params['/camera_perception_node/enable_signal'].value)
        self.assertNotIn('pointpainting_node', names)
        self.assertFalse(config.params['/world_model_node/pidnet_enabled'].value)

    def test_both_models_share_one_camera_node_and_yolo_environment(self):
        config = self.load()
        camera = [n for n in config.nodes if n.name == 'camera_perception_node']
        self.assertEqual(len(camera), 1)
        self.assertIn('morai-yolo/bin/python', camera[0].launch_prefix)
        self.assertTrue(config.params['/camera_perception_node/enable_semantic'].value)
        self.assertTrue(config.params['/camera_perception_node/enable_signal'].value)
        for pidnet, yolo in ((True, False), (False, True), (False, False)):
            config = self.load(['pidnet_enabled:='+str(pidnet).lower(),
                                'camera3_yolo:='+str(yolo).lower()])
            self.assertEqual(sum(n.name == 'camera_perception_node' for n in config.nodes),
                             int(pidnet or yolo))
            if pidnet or yolo:
                self.assertEqual(config.params['/camera_perception_node/enable_semantic'].value, pidnet)
                self.assertEqual(config.params['/camera_perception_node/enable_signal'].value, yolo)
