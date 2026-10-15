"""Regression evidence: official checkpoint amendment displayed 2026-10-07."""
import copy
from pathlib import Path
import unittest

import yaml

from hd_map_pkg.lane_rddf import read_route
from hd_map_pkg.runtime_map import build_static_map


class OfficialCheckpointsTest(unittest.TestCase):
    def test_amended_checkpoint_reaches_map_and_changes_map_identity(self):
        root = Path(__file__).resolve().parents[3]
        config = root / 'src/ros_architecture_pkg/config/map'
        checkpoints = yaml.safe_load((config / 'checkpoints.yaml').read_text())
        policy = yaml.safe_load((config / 'course_speed_policy.yaml').read_text())
        # Official amendment, not a coordinate inferred from the reference route.
        expected = [73.55110931396484, -238.7718963623047, 28.71942138671875]
        self.assertEqual(checkpoints['points'][10], expected)
        self.assertEqual(len(checkpoints['points']), 15)
        self.assertEqual(checkpoints['radius_m'], 3.0)
        route = read_route(root / '참고파일들/2026_molit_comp_global_path (3).txt')
        rddf = dict(graph=dict(longitudinal_connections=[], lane_changes=[]), lanes=[],
                    forbidden_boundaries=[], forbidden_boundary_ids=[], source_hashes={})
        current = build_static_map(route, rddf, checkpoints, policy, policy['reference']['sha256'])
        old = copy.deepcopy(checkpoints)
        old['points'][10] = [75.414203703, -217.855053961, 28.382283641]
        previous = build_static_map(route, rddf, old, policy, policy['reference']['sha256'])
        self.assertEqual(current['checkpoints'][10], expected)
        self.assertEqual(current['checkpoint_radius_m'], 3.0)
        self.assertNotEqual(current['map_id'], previous['map_id'])
        self.assertEqual(current['lanes'], previous['lanes'])
