#include <gtest/gtest.h>
#include "planner_status_text.h"

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(PlannerStatusText, ProducerModesAndStopAreSeparate) {
  EXPECT_EQ("Planner: Hybrid A* | Z2 | STOP", visualization_pkg::plannerStatusText(
      "planner_mode=hybrid_astar; zone=Z2; status=no_path; selected=stop; simulator_closed_loop_unverified", true));
  EXPECT_EQ("Planner: Frenet (RDDF) | Z3", visualization_pkg::plannerStatusText(
      "planner_mode=frenet; zone=Z3; selected=keep", false));
  EXPECT_EQ("Planner: Hybrid A* | Z5", visualization_pkg::plannerStatusText(
      "planner_mode=hybrid_astar; zone=Z5; status=success", false));
}
TEST(PlannerStatusText, MissingOrUnknownModeDoesNotGuess) {
  EXPECT_EQ("Planner: WAITING", visualization_pkg::plannerStatusText("waiting_for_map_route_world_localization", true));
  EXPECT_EQ("Planner: WAITING", visualization_pkg::plannerStatusText("unusable_or_stale_planning_inputs", true));
  EXPECT_EQ("Planner: UNKNOWN | Z1", visualization_pkg::plannerStatusText("planner_mode=new; zone=Z1", false));
}
