#include <zms/contract/Contract.hpp>
#include <zms/navigation/GridPlanner.hpp>
#include <zms/navigation/Navigation.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stop_token>

TEST(Contract, RejectsUnknownVersionAndUnknownFields) {
  const nlohmann::json valid = {
      {"contract", 1},
      {"command_id", "cmd-1"},
      {"client_id", "cli"},
      {"waypoints",
       nlohmann::json::array({{{"x", 1.0}, {"y", 0.0}, {"yaw", 0.0}}})}};
  EXPECT_EQ(zms::contract::parseGoal(valid).waypoints.size(), 1);
  auto wrongVersion = valid;
  wrongVersion["contract"] = 2;
  EXPECT_THROW(zms::contract::parseGoal(wrongVersion), std::invalid_argument);
  auto unknownField = valid;
  unknownField["topic"] = "other";
  EXPECT_THROW(zms::contract::parseGoal(unknownField), std::invalid_argument);
}

TEST(Contract, RejectsInvalidPrefix) {
  EXPECT_EQ(zms::contract::key("robot1/", "odom"), "robot1/odom");
  EXPECT_THROW(zms::contract::key("/robot1", "odom"), std::invalid_argument);
  EXPECT_THROW(zms::contract::key("robot*", "odom"), std::invalid_argument);
}

TEST(Contract, RoundTripsNavigationAndCancel) {
  const zms::contract::Goal goal{"goal-1", "client-1", {{1.0, 2.0, 0.5}}};
  const auto parsedGoal = zms::contract::parseGoal(zms::contract::toJson(goal));
  EXPECT_EQ(parsedGoal.commandId, goal.commandId);
  EXPECT_EQ(parsedGoal.clientId, goal.clientId);
  ASSERT_EQ(parsedGoal.waypoints.size(), 1);
  EXPECT_DOUBLE_EQ(parsedGoal.waypoints.front().yaw, 0.5);
  const zms::contract::Cancel cancel{"goal-1", "client-1"};
  EXPECT_EQ(zms::contract::parseCancel(zms::contract::toJson(cancel)).commandId,
            cancel.commandId);
}

TEST(Contract, RoundTripsManipulationTargets) {
  const zms::contract::Target pose{"pose-1", "client-1",
                                   zms::contract::PoseTarget{0.2, 0.1, 0.4, 0.0,
                                                             0.0, 0.0, 1.0,
                                                             "base", "tool"}};
  const auto parsedPose =
      zms::contract::parseTarget(zms::contract::toJson(pose));
  ASSERT_TRUE(
      std::holds_alternative<zms::contract::PoseTarget>(parsedPose.value));
  EXPECT_EQ(
      std::get<zms::contract::PoseTarget>(parsedPose.value).endEffectorFrame,
      "tool");

  const zms::contract::Target joints{
      "joints-1", "client-1",
      zms::contract::JointTarget{{"joint_1", "joint_2"}, {0.3, -0.2}}};
  const auto parsedJoints =
      zms::contract::parseTarget(zms::contract::toJson(joints));
  ASSERT_TRUE(
      std::holds_alternative<zms::contract::JointTarget>(parsedJoints.value));
  EXPECT_DOUBLE_EQ(
      std::get<zms::contract::JointTarget>(parsedJoints.value).positions.at(1),
      -0.2);
}

TEST(Contract, RejectsUnsafeManipulationTargets) {
  auto request = zms::contract::toJson(zms::contract::Target{
      "target-1", "client-1", zms::contract::JointTarget{{"joint_1"}, {0.0}}});
  request["joints"]["name"].push_back("joint_1");
  request["joints"]["position"].push_back(1.0);
  EXPECT_THROW(zms::contract::parseTarget(request), std::invalid_argument);
  request = zms::contract::toJson(zms::contract::Target{
      "target-2", "client-1", zms::contract::PoseTarget{}});
  request["pose"]["qw"] = 0.0;
  EXPECT_THROW(zms::contract::parseTarget(request), std::invalid_argument);
  request = zms::contract::toJson(zms::contract::Target{
      "target-3", "client-1", zms::contract::JointTarget{{"joint_1"}, {0.0}}});
  request["base_frame"] = "base";
  EXPECT_THROW(zms::contract::parseTarget(request), std::invalid_argument);
}

TEST(Navigation, ReachesWaypointAndLimitsAcceleration) {
  const std::vector<zms::contract::Waypoint> route = {{1.0, 0.0, 0.0}};
  const zms::navigation::Limits limits;
  const auto first =
      zms::navigation::follow({0.0, 0.0, 0.0}, route, 0, {}, limits, 0.1);
  EXPECT_FALSE(first.reached);
  EXPECT_LE(first.velocity.linear, limits.maxLinearAcceleration * 0.1);
  const auto final = zms::navigation::follow({1.0, 0.0, 0.0}, route, 0,
                                             first.velocity, limits, 0.1);
  EXPECT_TRUE(final.reached);
  EXPECT_DOUBLE_EQ(final.velocity.linear, 0.0);
}

TEST(Navigation, SlowsNearWaypoint) {
  const std::vector<zms::contract::Waypoint> route = {{1.0, 0.0, 0.0}};
  const zms::navigation::Limits limits;
  const auto far =
      zms::navigation::follow({0.0, 0.0, 0.0}, route, 0, {}, limits, 1.0);
  const auto near =
      zms::navigation::follow({0.8, 0.0, 0.0}, route, 0, {}, limits, 1.0);
  EXPECT_LT(near.velocity.linear, far.velocity.linear);
  EXPECT_LE(near.velocity.linear, 0.16);
}

TEST(Navigation, GridPathSkipsIntermediateYawAlignment) {
  const std::vector<zms::contract::Waypoint> route = {
      {0.0, 0.0, std::numbers::pi}, {1.0, 0.0, 0.0}};
  const zms::navigation::Limits limits;
  const auto step = zms::navigation::follow({0.0, 0.0, 0.0}, route, 0, {},
                                            limits, 0.1, false);
  EXPECT_EQ(step.waypointIndex, 1);
  EXPECT_FALSE(step.reached);
}

TEST(Navigation, StopsForForwardObstacleOnly) {
  const std::vector<double> ranges = {0.05, 10.0, 0.3, 0.4, 0.05};
  const double nearest = zms::navigation::nearestForwardObstacle(
      ranges, -std::numbers::pi, std::numbers::pi / 2.0, 0.1, 10.0);
  EXPECT_DOUBLE_EQ(nearest, 0.3);
  EXPECT_THROW(
      zms::navigation::nearestForwardObstacle(ranges, 0.0, 0.0, 0.1, 10.0),
      std::invalid_argument);
}

TEST(GridPlanner, RoutesThroughGapAndRejectsBlockedGoal) {
  nlohmann::json cells = nlohmann::json::array();
  for (int y = 0; y < 10; ++y) {
    for (int x = 0; x < 10; ++x) {
      cells.push_back(x == 4 && y != 8 ? 100 : 0);
    }
  }
  const auto grid = zms::navigation::parseGrid(
      {{"contract", 1},
       {"width", 10},
       {"height", 10},
       {"resolution", 1.0},
       {"origin", {{"x", 0.0}, {"y", 0.0}, {"yaw", 0.0}}},
       {"data", cells}});
  const auto route =
      zms::navigation::planGrid(grid, {1.5, 1.5, 0.0}, {8.5, 1.5, 0.0}, 0.0);
  ASSERT_FALSE(route.empty());
  EXPECT_TRUE(std::any_of(route.begin(), route.end(),
                          [](const auto &point) { return point.y >= 8.0; }));
  EXPECT_DOUBLE_EQ(route.back().x, 8.5);
  EXPECT_THROW(
      zms::navigation::planGrid(grid, {1.5, 1.5, 0.0}, {4.5, 1.5, 0.0}, 0.0),
      std::invalid_argument);
}

TEST(GridPlanner, StopsCancelledPlanning) {
  const zms::navigation::OccupancyGrid grid{
      10, 10, 1.0, {}, std::vector<std::int8_t>(100, 0)};
  std::stop_source source;
  source.request_stop();
  EXPECT_THROW(zms::navigation::planGrid(grid, {1.5, 1.5, 0.0}, {8.5, 8.5, 0.0},
                                         0.0, source.get_token()),
               std::runtime_error);
}

TEST(GridPlanner, EndsAtExactGoalWithoutCellCenterBacktrack) {
  const zms::navigation::OccupancyGrid grid{
      15, 10, 0.2, {0.0, -1.0, 0.0}, std::vector<std::int8_t>(150, 0)};
  const auto route =
      zms::navigation::planGrid(grid, {0.9, 0.0, 0.0}, {2.0, 0.0, 0.0}, 0.0);
  ASSERT_FALSE(route.empty());
  EXPECT_DOUBLE_EQ(route.back().x, 2.0);
  EXPECT_DOUBLE_EQ(route.back().y, 0.0);
  EXPECT_FALSE(std::any_of(route.begin(), route.end(), [](const auto &point) {
    return std::abs(point.x - 2.1) < 0.001 && std::abs(point.y - 0.1) < 0.001;
  }));
}
