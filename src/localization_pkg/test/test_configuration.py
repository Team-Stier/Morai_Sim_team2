#!/usr/bin/env python3
"""Localization 설정 파일과 launch의 정적 계약을 검사한다."""

from pathlib import Path
import math
import unittest
import xml.etree.ElementTree as ElementTree

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_DIR / "config"
LAUNCH_FILE = PACKAGE_DIR / "launch" / "localization.launch"
SOURCE_DIR = PACKAGE_DIR / "src"


class ConfigurationContractTest(unittest.TestCase):
    """YAML, launch, package.xml 사이의 이름·수치·의존성 계약을 검증한다."""

    @classmethod
    def setUpClass(cls):
        """모든 테스트가 공유할 localization 설정 파일을 한 번 읽는다."""
        cls.topics = cls._load_yaml("topics.yaml")
        cls.config = cls._load_yaml("localization.yaml")
        cls.local_ekf = cls._load_yaml("ekf_local.yaml")["/molit_local_ekf"]
        cls.global_ekf = cls._load_yaml("ekf_global.yaml")["/molit_global_ekf"]

    @staticmethod
    def _load_yaml(name):
        """config 디렉터리의 YAML 파일 하나를 Python 객체로 반환한다."""
        with (CONFIG_DIR / name).open(encoding="utf-8") as stream:
            return yaml.safe_load(stream)

    def test_global_path_output_contract(self):
        """Global Route Manager 입력 topic, frame과 공식 투영 원점을 확인한다."""
        self.assertEqual(self.topics["topics"]["output_odometry"],
                         "/localization/odometry")
        self.assertEqual(self.config["frames"],
                         {"map": "map", "odom": "odom", "base_link": "base_link"})
        self.assertEqual(self.config["projection"]["utm_zone"], 52)
        self.assertTrue(self.config["projection"]["north_hemisphere"])
        self.assertEqual(self.config["projection"]["origin_easting_m"], 302595.0)
        self.assertEqual(self.config["projection"]["origin_northing_m"], 4124145.0)
        self.assertEqual(self.config["projection"]["origin_altitude_m"], 0.0)

    def test_topics_are_absolute_and_non_empty(self):
        """모든 설정 topic이 비어 있지 않은 전역 ROS 이름인지 확인한다."""
        for topic in self.topics["topics"].values():
            self.assertTrue(topic)
            self.assertTrue(topic.startswith("/"))

    def test_ekf_sensor_selection_vectors_have_expected_length(self):
        """robot_localization sensor 선택 벡터가 15개 state와 맞는지 확인한다."""
        for filter_config in (self.local_ekf, self.global_ekf):
            for key, value in filter_config.items():
                if key.endswith("_config"):
                    self.assertEqual(len(value), 15, key)

    def test_ekf_fuses_only_required_planar_measurements(self):
        """두 EKF가 설계에서 정한 2D IMU·속도·GPS 성분만 융합하는지 확인한다."""
        self.assertEqual(
            self.local_ekf["imu0_config"],
            [False, False, False, False, False, True, False, False, False,
             False, False, True, False, False, False])
        self.assertEqual(
            self.global_ekf["imu0_config"], self.local_ekf["imu0_config"])
        self.assertEqual(
            self.local_ekf["twist0_config"],
            [False, False, False, False, False, False, True, True, False,
             False, False, False, False, False, False])
        self.assertEqual(
            self.global_ekf["twist0_config"], self.local_ekf["twist0_config"])
        self.assertEqual(
            self.global_ekf["pose0_config"],
            [True, True, False, False, False, False, False, False, False,
             False, False, False, False, False, False])

    def test_global_ekf_initial_covariance_allows_prompt_gps_anchor(self):
        """Global EKF가 첫 GPS anchor를 빠르게 반영할 초기 covariance인지 확인한다."""
        covariance = self.global_ekf["initial_estimate_covariance"]
        self.assertEqual(len(covariance), 15 * 15)
        self.assertTrue(all(math.isfinite(value) for value in covariance))
        for row in range(15):
            for column in range(15):
                value = covariance[row * 15 + column]
                if row == column:
                    self.assertGreater(value, 0.0)
                else:
                    self.assertEqual(value, 0.0)
        self.assertGreaterEqual(covariance[0], 100.0)
        self.assertGreaterEqual(covariance[16], 100.0)

    def test_tf_ownership_matches_dual_ekf_contract(self):
        """Local과 Global EKF의 world frame 및 TF 소유권이 분리됐는지 확인한다."""
        self.assertEqual(self.local_ekf["world_frame"], "odom")
        self.assertTrue(self.local_ekf["publish_tf"])
        self.assertEqual(self.global_ekf["world_frame"], "map")
        self.assertTrue(self.global_ekf["publish_tf"])
        self.assertEqual(self.local_ekf["odom_frame"], "odom")
        self.assertEqual(self.local_ekf["base_link_frame"], "base_link")
        self.assertEqual(self.global_ekf["map_frame"], "map")
        self.assertEqual(self.global_ekf["odom_frame"], "odom")

    def test_ekf_topics_match_normalized_topic_contract(self):
        """두 EKF 입력 topic이 공통 정규화 topic 설정과 일치하는지 확인한다."""
        topic_values = self.topics["topics"]
        local_filter_topics = self.topics["/molit_local_ekf"]
        global_filter_topics = self.topics["/molit_global_ekf"]
        self.assertEqual(local_filter_topics["imu0"], topic_values["input_imu"])
        self.assertEqual(local_filter_topics["twist0"],
                         topic_values["input_vehicle_twist"])
        self.assertEqual(global_filter_topics["imu0"], topic_values["input_imu"])
        self.assertEqual(global_filter_topics["twist0"],
                         topic_values["input_vehicle_twist"])
        self.assertEqual(global_filter_topics["pose0"], topic_values["gps_map_pose"])
        for filter_config in (self.local_ekf, self.global_ekf):
            self.assertEqual(filter_config["frequency"], 20.0)
            self.assertTrue(filter_config["two_d_mode"])

    def test_gps_projector_names_and_thresholds_are_configured(self):
        """GPS projector의 service와 모든 수치 임곗값 범위를 확인한다."""
        service_name = self.topics["services"]["global_set_pose"]
        self.assertTrue(service_name.startswith("/"))

        positive_values = [
            self.config["runtime"]["gps_timeout_sec"],
            self.config["runtime"]["gps_message_max_age_sec"],
            self.config["runtime"]["gps_state_timeout_sec"],
            self.config["runtime"]["global_odometry_timeout_sec"],
            self.config["runtime"]["timeout_check_period_sec"],
            self.config["validation"]["min_quaternion_norm"],
            self.config["validation"]["global_anchor_max_error_m"],
            self.config["reacquisition"]["set_pose_call_timeout_sec"],
            self.config["gps_covariance"]["fallback_xy_variance_m2"],
            self.config["gps_covariance"]["unobserved_variance"],
            self.config["reacquisition"]["reset_xy_variance_m2"],
            self.config["reacquisition"]["reset_unobserved_variance"],
        ]
        nonnegative_values = [
            self.config["runtime"]["max_future_stamp_sec"],
            self.config["reacquisition"]["max_innovation_m"],
            self.config["reacquisition"]["stable_return_radius_m"],
        ]
        for value in positive_values:
            self.assertTrue(math.isfinite(value))
            self.assertGreater(value, 0.0)
        for value in nonnegative_values:
            self.assertTrue(math.isfinite(value))
            self.assertGreaterEqual(value, 0.0)
        self.assertGreater(
            self.config["reacquisition"]["required_consecutive_fixes"], 0)

    def test_supervisor_frames_and_thresholds_are_configured(self):
        """Supervisor가 사용할 sensor frame, 주기와 quaternion 허용값을 확인한다."""
        self.assertEqual(
            self.config["sensor_frames"],
            {"imu": "base_link", "vehicle_twist": "base_link"})
        self.assertEqual(self.config["runtime"]["publish_rate_hz"], 20.0)
        self.assertTrue(math.isfinite(
            self.config["runtime"]["sensor_timeout_sec"]))
        self.assertGreater(self.config["runtime"]["sensor_timeout_sec"], 0.0)
        tolerance = self.config["validation"][
            "quaternion_unit_norm_tolerance"]
        self.assertTrue(math.isfinite(tolerance))
        self.assertGreater(tolerance, 0.0)
        self.assertLess(tolerance, 1.0)

    def test_receipt_deadlines_use_monotonic_ros_time(self):
        """수신 timeout 코드가 시스템 시각이 아닌 monotonic ROS 시간을 쓰는지 확인한다."""
        for source_name in (
                "gps_projector_node.cpp", "localization_supervisor_node.cpp"):
            source = (SOURCE_DIR / source_name).read_text(encoding="utf-8")
            self.assertNotIn("ros::WallTime", source, source_name)
            self.assertNotIn("ros::WallTimer", source, source_name)
            self.assertIn("ros::SteadyTime", source, source_name)
            self.assertIn("ros::SteadyTimer", source, source_name)

    def test_launch_loads_wrapped_yaml_at_root_and_matches_topic_contract(self):
        """launch가 모든 YAML을 root에 읽고 EKF remap을 topic 계약과 맞추는지 확인한다."""
        launch = ElementTree.parse(LAUNCH_FILE).getroot()
        loaded_files = {
            Path(parameter.attrib["file"]).name
            for parameter in launch.findall("rosparam")
        }
        self.assertEqual(
            loaded_files,
            {"topics.yaml", "localization.yaml", "ekf_local.yaml",
             "ekf_global.yaml"})

        nodes = {node.attrib["name"]: node for node in launch.findall("node")}
        local_remaps = {
            remap.attrib["from"]: remap.attrib["to"]
            for remap in nodes["molit_local_ekf"].findall("remap")
        }
        global_remaps = {
            remap.attrib["from"]: remap.attrib["to"]
            for remap in nodes["molit_global_ekf"].findall("remap")
        }
        self.assertEqual(
            local_remaps["odometry/filtered"],
            self.topics["topics"]["local_filtered_odometry"])
        self.assertEqual(local_remaps["set_pose"],
                         "/molit_local_ekf/set_pose")
        self.assertEqual(
            global_remaps["odometry/filtered"],
            self.topics["topics"]["global_filtered_odometry"])
        self.assertEqual(global_remaps["set_pose"],
                         self.topics["services"]["global_set_pose"])

    def test_ros_graph_test_dependencies_are_declared(self):
        """통합 테스트가 import하는 ROS graph 도구가 test_depend에 있는지 확인한다."""
        package = ElementTree.parse(PACKAGE_DIR / "package.xml").getroot()
        test_dependencies = {
            dependency.text for dependency in package.findall("test_depend")
        }
        for dependency in ("rosgraph", "rosnode", "rosservice"):
            self.assertIn(dependency, test_dependencies)


if __name__ == "__main__":
    unittest.main()


