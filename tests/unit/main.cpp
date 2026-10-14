#include <zms/contract/Contract.hpp>
#include <zms/manipulation/JointModel.hpp>
#include <zms/manipulation/JointMotion.hpp>
#include <zms/navigation/GridPlanner.hpp>
#include <zms/navigation/Navigation.hpp>
#include <zms/navigation/PlanningWorker.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <stop_token>
#include <thread>

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

TEST(Manipulation, LoadsUrdfChainAndChecksTargets) {
  const auto model = zms::manipulation::JointModel::fromUrdf(
      "config/examples/two_joint_arm.urdf", "base", "tool");
  ASSERT_EQ(model.joints().size(), 2);
  EXPECT_EQ(model.joints().front().name, "joint_1");
  EXPECT_EQ(model.joints().back().name, "joint_2");
  const auto positions = model.orderedPositions(
      zms::contract::JointTarget{{"joint_2", "joint_1"}, {-0.2, 0.3}});
  ASSERT_EQ(positions.size(), 2);
  EXPECT_DOUBLE_EQ(positions.front(), 0.3);
  EXPECT_DOUBLE_EQ(positions.back(), -0.2);
  EXPECT_THROW(model.orderedPositions(zms::contract::JointTarget{
                   {"joint_1", "joint_2"}, {2.0, 0.0}}),
               std::invalid_argument);
  EXPECT_THROW(model.orderedPositions(zms::contract::JointTarget{
                   {"joint_1", "unknown"}, {0.0, 0.0}}),
               std::invalid_argument);
  EXPECT_THROW(zms::manipulation::JointModel::fromUrdf(
                   "config/examples/two_joint_arm.urdf", "tool", "base"),
               std::invalid_argument);
}

TEST(Manipulation, ReadsChainJointsFromFullJointState) {
  const auto model = zms::manipulation::JointModel::fromUrdf(
      "config/examples/two_joint_arm.urdf", "base", "tool");
  const auto measured = model.measuredPositions(
      {"wheel_left", "joint_2", "gripper", "joint_1"}, {4.0, -0.2, 0.01, 0.3});
  ASSERT_TRUE(measured.has_value());
  ASSERT_EQ(measured->size(), 2);
  EXPECT_DOUBLE_EQ(measured->front(), 0.3);
  EXPECT_DOUBLE_EQ(measured->back(), -0.2);
  const double beyondLimit = model.joints().front().upper + 0.01;
  const auto overshoot =
      model.measuredPositions({"joint_1", "joint_2"}, {beyondLimit, 0.0});
  ASSERT_TRUE(overshoot.has_value());
  EXPECT_DOUBLE_EQ(overshoot->front(), beyondLimit);
  EXPECT_FALSE(model.measuredPositions({"joint_1", "wheel_left"}, {0.0, 0.0})
                   .has_value());
  EXPECT_FALSE(
      model
          .measuredPositions({"joint_1", "joint_2", "joint_1"}, {0.0, 0.0, 0.0})
          .has_value());
  EXPECT_FALSE(
      model.measuredPositions({"joint_1", "joint_2"}, {0.0}).has_value());
}

TEST(Manipulation, JointStepBrakesWithinAccelerationLimit) {
  const double maxVelocity = 1.0;
  const double maxAcceleration = 2.0;
  const double elapsed = 0.05;
  for (const double goal : {1.0, -0.7, 0.003}) {
    zms::manipulation::JointStep state;
    double peakVelocityChange = 0.0;
    int steps = 0;
    while ((state.position != goal || state.velocity != 0.0) && steps < 1000) {
      const auto next =
          zms::manipulation::stepTowards(state.position, state.velocity, goal,
                                         maxVelocity, maxAcceleration, elapsed);
      peakVelocityChange = std::max(peakVelocityChange,
                                    std::abs(next.velocity - state.velocity));
      EXPECT_LE(std::abs(next.velocity), maxVelocity + 1e-12);
      EXPECT_LE(std::abs(next.position), std::abs(goal) + 1e-12);
      state = next;
      ++steps;
    }
    EXPECT_LT(steps, 1000);
    EXPECT_DOUBLE_EQ(state.position, goal);
    EXPECT_LE(peakVelocityChange, maxAcceleration * elapsed + 1e-9);
  }
}

TEST(PlanningWorker, ReportsSubmittedWaypointForEachRoutePoint) {
  auto grid = std::make_shared<zms::navigation::OccupancyGrid>(
      zms::navigation::OccupancyGrid{
          10, 10, 1.0, {}, std::vector<std::int8_t>(100, 0)});
  zms::navigation::PlanningWorker worker;
  const auto generation = worker.submit(
      {grid, {0.5, 0.5, 0.0}, {{5.5, 3.5, 0.0}, {8.5, 8.5, 0.0}}, 0.0});
  std::optional<zms::navigation::PlanOutcome> outcome;
  for (int attempt = 0; attempt < 500 && !outcome; ++attempt) {
    outcome = worker.poll();
    if (!outcome) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  ASSERT_TRUE(outcome.has_value());
  EXPECT_EQ(outcome->generation, generation);
  ASSERT_TRUE(outcome->error.empty()) << outcome->error;
  ASSERT_EQ(outcome->waypointIndices.size(), outcome->route.size());
  EXPECT_EQ(outcome->waypointIndices.front(), 0);
  EXPECT_EQ(outcome->waypointIndices.back(), 1);
  EXPECT_TRUE(std::is_sorted(outcome->waypointIndices.begin(),
                             outcome->waypointIndices.end()));
  const auto firstGoal = std::find(outcome->waypointIndices.begin(),
                                   outcome->waypointIndices.end(), 1) -
                         outcome->waypointIndices.begin() - 1;
  EXPECT_DOUBLE_EQ(outcome->route[firstGoal].x, 5.5);
  EXPECT_DOUBLE_EQ(outcome->route[firstGoal].y, 3.5);
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

TEST(Navigation, KeepsTurningAtGoalAfterSmallDrift) {
  const std::vector<zms::contract::Waypoint> route = {
      {1.0, 0.0, std::numbers::pi / 2.0}};
  const zms::navigation::Limits limits;
  const zms::navigation::Pose drifted{1.0 + limits.positionTolerance * 1.5, 0.0,
                                      0.0};
  const auto released =
      zms::navigation::follow(drifted, route, 0, {}, limits, 1.0, true, false);
  EXPECT_FALSE(released.aligning);
  const auto aligning =
      zms::navigation::follow(drifted, route, 0, {}, limits, 1.0, true, true);
  EXPECT_TRUE(aligning.aligning);
  EXPECT_FALSE(aligning.reached);
  EXPECT_DOUBLE_EQ(aligning.velocity.linear, 0.0);
  EXPECT_GT(aligning.velocity.angular, 0.0);
  const zms::navigation::Pose aligned{1.0 + limits.positionTolerance * 1.5, 0.0,
                                      std::numbers::pi / 2.0};
  EXPECT_TRUE(
      zms::navigation::follow(aligned, route, 0, {}, limits, 1.0, true, true)
          .reached);
  const zms::navigation::Pose tooFar{1.0 + limits.positionTolerance * 3.0, 0.0,
                                     std::numbers::pi / 2.0};
  const auto beyond =
      zms::navigation::follow(tooFar, route, 0, {}, limits, 1.0, true, true);
  EXPECT_FALSE(beyond.reached);
  EXPECT_FALSE(beyond.aligning);
}

TEST(Navigation, ProgressMonitorFailsOnlyWithoutImprovement) {
  zms::navigation::ProgressMonitor monitor(2.0);
  EXPECT_FALSE(monitor.stalled(0, 1.0, 0.5, 0.0));
  EXPECT_FALSE(monitor.stalled(0, 1.0, 0.5, 1.9));
  EXPECT_TRUE(monitor.stalled(0, 1.0, 0.5, 2.1));
  monitor.reset();
  EXPECT_FALSE(monitor.stalled(0, 1.0, 0.5, 10.0));
  EXPECT_FALSE(monitor.stalled(0, 0.9, 0.5, 11.5));
  EXPECT_FALSE(monitor.stalled(0, 0.9, 0.3, 13.0));
  EXPECT_FALSE(monitor.stalled(0, 0.9, 0.3, 14.9));
  EXPECT_TRUE(monitor.stalled(0, 0.89, 0.29, 15.1));
  EXPECT_FALSE(monitor.stalled(1, 2.0, 0.0, 15.2));
  EXPECT_THROW(zms::navigation::ProgressMonitor(0.0), std::invalid_argument);
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
