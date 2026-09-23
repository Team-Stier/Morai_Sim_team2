"""Check unlimited GPS blackout policy independently of the live ROS master."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import rospy
import yaml

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('wall_validity_node',ROOT/'src/localization_pkg/src/localization_estimator_node.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class WallValidityTest(unittest.TestCase):
    def node(self):
        n=module.LocalizationNode.__new__(module.LocalizationNode)
        n.epoch=0;n.wall_matcher=object();n.wall_config=SimpleNamespace(enabled=True)
        n.timing=yaml.safe_load((ROOT/'src/ros_architecture_pkg/config/timestamp/timestamp_contract.yaml').read_text())['development_localization_profile']
        n.core=SimpleNamespace(last_gps_stamp=80.,last_wall_stamp=99.9,initialized=False,
                              wall_diagnostic='wall matched',gps_diagnostic='',last_rejection='')
        n.arrival={'imu':49.99,'gps':30.};n.reason='test'
        n.output_stamp=rospy.Time.from_sec(99.95)
        n.latest=dict(map_position_stddev=2.,local_position_stddev=4.,yaw_stddev=.1)
        n.status_pub=SimpleNamespace(publish=lambda m:setattr(n,'published',m))
        return n

    def test_no_gps_age_or_uncertainty_limit_with_or_without_wall_support(self):
        for age in (15.01,60.01,3600.,86400.):
            for wall_support in (False,True):
                n=self.node();now=100000.;n.core.last_gps_stamp=now-age
                n.core.last_wall_stamp=now-.1 if wall_support else None
                n.wall_matcher=object() if wall_support else None
                n.latest['map_position_stddev']=1000.
                n.output_stamp=rospy.Time.from_sec(now-.05)
                n.publish_status(rospy.Time.from_sec(now),50.,False)
                with self.subTest(age=age,wall_support=wall_support):
                    self.assertTrue(n.published.map_pose_valid)
                    self.assertTrue(n.published.local_odometry_valid)
                    self.assertFalse(n.published.gps_fix_valid)
                    self.assertEqual(n.published.mode,n.published.DEAD_RECKONING)
                    self.assertEqual(n.published.ego_state_stamp,n.output_stamp)
                    self.assertEqual(n.published.map_position_stddev_m,1000.)
                    self.assertTrue(n.published.stop_required)

    def test_sensor_clock_and_initialization_guards_remain(self):
        for fault in ('stale_imu','stale_estimate','future_estimate','zero_estimate','never_gps','no_estimate','clock_stall'):
            n=self.node()
            if fault=='stale_imu':n.arrival['imu']=49.
            if fault=='stale_estimate':n.output_stamp=rospy.Time(99)
            if fault=='future_estimate':n.output_stamp=rospy.Time(101)
            if fault=='zero_estimate':n.output_stamp=rospy.Time(0)
            if fault=='never_gps':n.core.last_gps_stamp=None
            if fault=='no_estimate':n.latest=None
            n.publish_status(rospy.Time(100),50.,fault=='clock_stall')
            with self.subTest(fault=fault):
                self.assertFalse(n.published.map_pose_valid)
                self.assertFalse(n.published.local_odometry_valid)

    def test_central_policy_has_no_inactive_legacy_duration_knobs(self):
        n=self.node()
        self.assertEqual(n.timing['gps_blackout_duration_limit'],'none')
        for key in ('max_dead_reckoning_sec','max_wall_aided_blackout_sec','wall_aided_max_position_stddev_m'):
            self.assertNotIn(key,n.timing)
