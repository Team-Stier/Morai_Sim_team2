#!/usr/bin/env python3

import copy
import math
from types import SimpleNamespace
import unittest

from vehicle_control_pkg.team1_adapter_core import (
    AdapterConfig,
    quaternion_from_yaw,
    validate_odometry,
    validate_raw_command,
    validate_trajectory,
)


def namespace(**values):
    return SimpleNamespace(**values)


def stamp(value):
    return namespace(to_sec=lambda: value)


def config_values():
    return {
        "trajectory_timeout_sec": 0.75,
        "odometry_timeout_sec": 0.20,
        "raw_command_timeout_sec": 0.20,
        "minimum_path_points": 2,
        "queue_size": 10,
        "maximum_accel_command": 0.40,
        "maximum_brake_command": 0.60,
        "maximum_steering_angle_rad": math.radians(40.0),
        "controller_target_speed_mps": 10.0 / 3.6,
    }


def trajectory():
    point_a = namespace(
        x_m=1.0,
        y_m=2.0,
        z_m=0.0,
        yaw_rad=0.1,
        curvature_1pm=0.01,
        s_m=0.0,
        target_speed_mps=3.0,
    )
    point_b = namespace(
        x_m=2.0,
        y_m=2.1,
        z_m=0.0,
        yaw_rad=0.2,
        curvature_1pm=0.02,
        s_m=1.1,
        target_speed_mps=3.0,
    )
    return namespace(
        STATUS_VALID=0,
        header=namespace(frame_id="map", stamp=stamp(9.8)),
        valid_until=stamp(10.5),
        status=0,
        minimum_boundary_clearance_m=0.12,
        points=[point_a, point_b],
    )


def odometry():
    vector = lambda x=0.0, y=0.0, z=0.0: namespace(x=x, y=y, z=z)
    pose = namespace(
        position=vector(1.0, 2.0, 0.0),
        orientation=namespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    twist = namespace(linear=vector(2.0, 0.0, 0.0), angular=vector())
    return namespace(
        header=namespace(frame_id="map", stamp=stamp(9.9)),
        child_frame_id="base_link",
        pose=namespace(pose=pose, covariance=[0.0] * 36),
        twist=namespace(twist=twist, covariance=[0.0] * 36),
    )


class AdapterCoreTest(unittest.TestCase):
    def setUp(self):
        self.config = AdapterConfig.from_mapping(config_values())

    def test_config_rejects_missing_extra_and_non_integer_values(self):
        for mutate in (
            lambda values: values.pop("controller_target_speed_mps"),
            lambda values: values.update({"unknown": 1}),
            lambda values: values.update({"queue_size": 1.5}),
            lambda values: values.update({"minimum_path_points": 1}),
            lambda values: values.update({"maximum_accel_command": 1.1}),
            lambda values: values.update({"maximum_brake_command": 1.1}),
            lambda values: values.update(
                {"maximum_steering_angle_rad": math.radians(41.0)}
            ),
        ):
            values = config_values()
            mutate(values)
            with self.assertRaises(ValueError):
                AdapterConfig.from_mapping(values)

    def test_valid_trajectory_is_accepted(self):
        self.assertTrue(validate_trajectory(trajectory(), 10.0, self.config).valid)

    def test_fixed_controller_speed_fails_closed_for_slower_planner_points(self):
        value = trajectory()
        value.points[1].target_speed_mps = self.config.controller_target_speed_mps
        self.assertTrue(validate_trajectory(value, 10.0, self.config).valid)

        value.points[1].target_speed_mps = (
            self.config.controller_target_speed_mps - 1.0e-3
        )
        result = validate_trajectory(value, 10.0, self.config)
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "planner_speed_below_controller_target")

    def test_trajectory_faults_fail_closed(self):
        mutations = {
            "status": lambda value: setattr(value, "status", 1),
            "frame": lambda value: setattr(value.header, "frame_id", "odom"),
            "future": lambda value: setattr(value.header, "stamp", stamp(10.1)),
            "stale": lambda value: setattr(value.header, "stamp", stamp(9.0)),
            "expired": lambda value: setattr(value, "valid_until", stamp(10.0)),
            "clearance": lambda value: setattr(
                value, "minimum_boundary_clearance_m", 0.0
            ),
            "few_points": lambda value: setattr(value, "points", value.points[:1]),
            "nan": lambda value: setattr(value.points[1], "x_m", float("nan")),
            "arc_length": lambda value: setattr(value.points[1], "s_m", 0.0),
            "speed": lambda value: setattr(value.points[1], "target_speed_mps", -1.0),
            "planner_speed_cap": lambda value: setattr(
                value.points[1], "target_speed_mps", 2.0
            ),
            "duplicate": lambda value: (
                setattr(value.points[1], "x_m", value.points[0].x_m),
                setattr(value.points[1], "y_m", value.points[0].y_m),
            ),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name):
                value = copy.deepcopy(trajectory())
                mutate(value)
                self.assertFalse(
                    validate_trajectory(value, 10.0, self.config).valid
                )

    def test_valid_odometry_is_accepted(self):
        self.assertTrue(validate_odometry(odometry(), 10.0, self.config).valid)

    def test_odometry_faults_fail_closed(self):
        mutations = {
            "frame": lambda value: setattr(value.header, "frame_id", "odom"),
            "child": lambda value: setattr(value, "child_frame_id", "base_footprint"),
            "future": lambda value: setattr(value.header, "stamp", stamp(10.1)),
            "stale": lambda value: setattr(value.header, "stamp", stamp(9.0)),
            "nan": lambda value: setattr(
                value.twist.twist.linear, "x", float("nan")
            ),
            "quaternion": lambda value: setattr(
                value.pose.pose,
                "orientation",
                namespace(x=0.0, y=0.0, z=0.0, w=0.0),
            ),
            "covariance": lambda value: setattr(value.pose, "covariance", [0.0]),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name):
                value = copy.deepcopy(odometry())
                mutate(value)
                self.assertFalse(validate_odometry(value, 10.0, self.config).valid)

    def test_raw_command_range_and_mutual_exclusion(self):
        valid = namespace(
            header=namespace(frame_id="base_link", stamp=stamp(9.9)),
            accel=0.2,
            brake=0.0,
            steering_angle_rad=0.1,
        )
        self.assertTrue(validate_raw_command(valid, 10.0, self.config).valid)
        for field, value in (
            ("accel", 0.41),
            ("brake", 0.61),
            ("steering_angle_rad", math.radians(41.0)),
        ):
            invalid = copy.deepcopy(valid)
            setattr(invalid, field, value)
            self.assertFalse(validate_raw_command(invalid, 10.0, self.config).valid)
        simultaneous = copy.deepcopy(valid)
        simultaneous.brake = 0.1
        self.assertFalse(
            validate_raw_command(simultaneous, 10.0, self.config).valid
        )
        wrong_frame = copy.deepcopy(valid)
        wrong_frame.header.frame_id = "map"
        self.assertFalse(validate_raw_command(wrong_frame, 10.0, self.config).valid)

    def test_yaw_conversion_is_finite_and_normalized(self):
        quaternion = quaternion_from_yaw(math.pi / 2.0)
        self.assertAlmostEqual(sum(value * value for value in quaternion), 1.0)
        with self.assertRaises(ValueError):
            quaternion_from_yaw(float("nan"))


if __name__ == "__main__":
    unittest.main()
