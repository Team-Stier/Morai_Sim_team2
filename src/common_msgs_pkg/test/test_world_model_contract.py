import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

import yaml


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/common_msgs_pkg/src"))
from common_msgs_pkg.world_model_validation import validate_world_model


def stamp(value):
    seconds = int(value)
    return NS(secs=seconds, nsecs=int(round((value - seconds) * 1.0e9)))


def tracked(track_id=1):
    return NS(
        CLASS_UNKNOWN=0,
        CLASS_PEDESTRIAN=1,
        CLASS_VEHICLE=2,
        CLASS_OTHER=3,
        TRACK_TENTATIVE=0,
        TRACK_CONFIRMED=1,
        TRACK_COASTING=2,
        track_id=track_id,
        source_stamp=stamp(10.0),
        source_frame_id="lidar_link",
        source_local_id=0,
        timestamp_provenance="ingress_fallback",
        calibration_id="development-lidar",
        calibration_verified=False,
        pose=NS(position=NS(x=12.0, y=2.0, z=0.5), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
        points=[NS(x=12., y=2., z=.5)],
        twist=NS(linear=NS(x=0.0, y=0.0, z=0.0), angular=NS(x=0.0, y=0.0, z=0.0)),
        velocity_valid=False,
        semantic_class=0,
        confidence=-1.0,
        track_state=1,
        observation_count=2,
        age_since_observation_sec=0.0,
        position_stddev_m=-1.0,
        velocity_stddev_mps=-1.0,
    )


def scene():
    return NS(
        header=NS(frame_id="map", stamp=stamp(10.0)),
        localization_reset_id=3,
        objects_valid=True,
        tracking_valid=True,
        objects_verified=False,
        map_context_valid=False,
        route_context_valid=False,
        occupancy_valid=False,
        free_space_valid=False,
        traffic_context_valid=False,
        planner_ready=False,
        objects=[tracked()],
        reason="development geometry only",
    )


class WorldModelContractTest(unittest.TestCase):
    def test_exact_wire_and_registry(self):
        config = ROOT / "src/ros_architecture_pkg/config"
        schema = yaml.safe_load((config / "messages/world_model_messages.yaml").read_text(encoding="utf-8"))
        contract = yaml.safe_load((config / "interface_contract.yaml").read_text(encoding="utf-8"))
        module = contract["contract_modules"]["world_model_messages"]
        self.assertEqual(module["path"], "messages/world_model_messages.yaml")
        self.assertEqual(module["required_contract_version"], schema["contract_version"])
        for name, entry in schema["messages"].items():
            path = ROOT / "src/common_msgs_pkg/msg" / (name + ".msg")
            self.assertEqual(path.read_text(encoding="utf-8"), entry["wire_definition"])
            registered = next(item for item in contract["messages"] if item["type"] == "common_msgs_pkg/" + name)
            self.assertIn("schema", registered)

    def test_development_scene_is_valid_but_not_planner_ready(self):
        value = scene()
        self.assertTrue(validate_world_model(value))
        with self.assertRaises(ValueError):
            validate_world_model(value, for_planning=True)

    def test_rejects_duplicate_track_and_time_mismatch(self):
        value = scene()
        value.objects.append(tracked())
        with self.assertRaises(ValueError):
            validate_world_model(value)
        value = scene()
        value.objects[0].age_since_observation_sec = 0.2
        with self.assertRaises(ValueError):
            validate_world_model(value)

    def test_planner_ready_requires_all_gates(self):
        value = scene()
        value.planner_ready = True
        with self.assertRaises(ValueError):
            validate_world_model(value)


if __name__ == "__main__":
    unittest.main()
