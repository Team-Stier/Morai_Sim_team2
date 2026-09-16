import copy
import importlib.util
from pathlib import Path
import unittest
import yaml
ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('static_tf', ROOT/'src/system_bringup_pkg/src/sensor_tf_publisher.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class StaticTest(unittest.TestCase):
    def setUp(self):
        config = ROOT/'src/ros_architecture_pkg/config/tf'
        self.frames = yaml.safe_load((config/'frame_contract.yaml').read_text())
        self.mounts = yaml.safe_load((config/'sensor_extrinsics.yaml').read_text())

    def test_only_approved_gps_imu_lidar(self):
        poses = module.approved_static_poses(self.frames, self.mounts)
        self.assertEqual({p[1] for p in poses}, {'gps_link', 'imu_link', 'lidar_link'})
        self.assertEqual(next(p[2] for p in poses if p[1] == 'gps_link'), [0., 0., 1.3])

    def test_gate_mismatch_cycle_and_multiple_parents_rejected(self):
        original = copy.deepcopy(self.frames)
        self.frames['transforms'][-1]['publish_enabled'] = False
        with self.assertRaises(ValueError):
            module.approved_static_poses(self.frames, self.mounts)
        self.frames = copy.deepcopy(original)
        self.frames['transforms'][0]['parent'] = 'base_link'
        with self.assertRaises(ValueError):
            module.approved_static_poses(self.frames, self.mounts)
        self.frames = original
        self.frames['transforms'].append(copy.deepcopy(original['transforms'][0]))
        with self.assertRaises(ValueError):
            module.approved_static_poses(self.frames, self.mounts)

    def test_lidar_pose_and_disabled_gate(self):
        poses = module.approved_static_poses(self.frames, self.mounts)
        lidar = next(p for p in poses if p[1] == 'lidar_link')
        self.assertEqual(lidar, ('base_link', 'lidar_link', [2.0, 0., 1.5], (0., 0., 0., 1.)))
        next(t for t in self.frames['transforms'] if t['child'] == 'lidar_link')['publish_enabled'] = False
        with self.assertRaises(ValueError):
            module.approved_static_poses(self.frames, self.mounts)

    def test_lidar_requires_coordinate_approval(self):
        self.mounts["source_evidence"]["user_confirmed_lidar_mount"]["user_confirmed_coordinate_alignment"] = False
        with self.assertRaises(ValueError):
            module.approved_static_poses(self.frames, self.mounts)
