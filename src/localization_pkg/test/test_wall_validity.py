"""Check central validity bounds without publishing to a ROS master."""
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

    def test_wall_aid_is_bounded_and_does_not_refresh_measurement_stamp(self):
        n=self.node();n.publish_status(rospy.Time(100),50.,False)
        self.assertTrue(n.published.map_pose_valid)
        self.assertEqual(n.published.mode,n.published.DEAD_RECKONING)
        self.assertEqual(n.published.ego_state_stamp,n.output_stamp)
        self.assertTrue(n.published.stop_required)
        for change in ('stale_scan','gps_age','uncertainty','stale_imu','never_gps'):
            n=self.node()
            if change=='stale_scan':n.core.last_wall_stamp=99.
            if change=='gps_age':n.core.last_gps_stamp=39.
            if change=='uncertainty':n.latest['map_position_stddev']=3.1
            if change=='stale_imu':n.arrival['imu']=49.
            if change=='never_gps':n.core.last_gps_stamp=None
            n.publish_status(rospy.Time(100),50.,False)
            with self.subTest(change=change):self.assertFalse(n.published.map_pose_valid)
