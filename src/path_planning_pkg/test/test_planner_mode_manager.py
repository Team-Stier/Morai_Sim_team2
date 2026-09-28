import unittest

from path_planning_pkg.planner_mode_manager import (
    FRENET,
    HYBRID_ASTAR,
    PlannerModeManager,
    PlannerZone,
)


def manager():
    return PlannerModeManager(2184.6117233360674, (
        PlannerZone("Z1", 0.0, 237.423, HYBRID_ASTAR),
        PlannerZone("Z2", 237.423, 635.113, HYBRID_ASTAR),
        PlannerZone("Z3", 635.113, 1118.7417511690987, HYBRID_ASTAR),
        PlannerZone("Z4", 1118.7417511690987, 1741.7209885112272, FRENET),
        PlannerZone("Z5", 1741.7209885112272, 2184.6117233360674, HYBRID_ASTAR),
    ))


class PlannerModeManagerTest(unittest.TestCase):
    def test_exact_boundaries_are_start_inclusive(self):
        subject = manager()
        self.assertEqual(subject.select(0.0).zone_id, "Z1")
        self.assertEqual(subject.select(237.423).zone_id, "Z2")
        self.assertEqual(subject.select(635.113).zone_id, "Z3")
        selection = subject.select(1118.7417511690987)
        self.assertEqual((selection.zone_id, selection.planner), ("Z4", FRENET))
        selection = subject.select(1741.7209885112272)
        self.assertEqual((selection.zone_id, selection.planner), ("Z5", HYBRID_ASTAR))
        self.assertEqual(subject.select(2184.6117233360674).zone_id, "Z5")

    def test_same_planner_zone_boundary_is_not_a_mode_change(self):
        subject = manager()
        subject.select(200.0)
        selection = subject.select(300.0)
        self.assertEqual(selection.zone_id, "Z2")
        self.assertFalse(selection.changed)

    def test_progress_does_not_reenter_previous_planner_zone(self):
        subject = manager()
        self.assertEqual(subject.select(1200.0).planner, FRENET)
        selection = subject.select(1100.0)
        self.assertEqual(selection.planner, FRENET)
        self.assertEqual(selection.progress_s, 1200.0)
        self.assertEqual(subject.select(1800.0).planner, HYBRID_ASTAR)
        self.assertEqual(subject.select(1700.0).planner, HYBRID_ASTAR)

    def test_reset_starts_a_new_run(self):
        subject = manager()
        subject.select(1800.0)
        subject.reset()
        selection = subject.select(10.0)
        self.assertEqual((selection.zone_id, selection.planner), ("Z1", HYBRID_ASTAR))

    def test_rejects_gap_or_overlap(self):
        with self.assertRaises(ValueError):
            PlannerModeManager(20.0, (
                PlannerZone("A", 0.0, 9.0, HYBRID_ASTAR),
                PlannerZone("B", 10.0, 20.0, FRENET),
            ))


if __name__ == "__main__":
    unittest.main()
