#include <zms/contract/Contract.hpp>
#include <zms/contract/ZenohClient.hpp>

#include <gtest/gtest.h>

#include <zenoh.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

namespace {

using Clock = std::chrono::steady_clock;
using Json = nlohmann::json;
using namespace std::chrono_literals;

class Process {
public:
  Process() = default;
  Process(const std::filesystem::path &program,
          const std::vector<std::string> &arguments) {
    start(program, arguments);
  }
  Process(const Process &) = delete;
  Process &operator=(const Process &) = delete;
  ~Process() { stop(); }

  void start(const std::filesystem::path &program,
             const std::vector<std::string> &arguments) {
    if (pid_ > 0) {
      throw std::logic_error("process is already running");
    }
    std::vector<std::string> values;
    values.push_back(program.string());
    values.insert(values.end(), arguments.begin(), arguments.end());
    std::vector<char *> pointers;
    for (auto &value : values) {
      pointers.push_back(value.data());
    }
    pointers.push_back(nullptr);
    pid_ = fork();
    if (pid_ < 0) {
      throw std::runtime_error("fork failed");
    }
    if (pid_ == 0) {
      execv(program.c_str(), pointers.data());
      _exit(127);
    }
  }

  void stop() {
    if (pid_ <= 0) {
      return;
    }
    int status = 0;
    if (waitpid(pid_, &status, WNOHANG) == pid_) {
      pid_ = -1;
      return;
    }
    kill(pid_, SIGTERM);
    const auto deadline = Clock::now() + 3s;
    while (Clock::now() < deadline) {
      const auto waited = waitpid(pid_, &status, WNOHANG);
      if (waited == pid_ || waited == -1) {
        pid_ = -1;
        return;
      }
      std::this_thread::sleep_for(20ms);
    }
    kill(pid_, SIGKILL);
    waitpid(pid_, &status, 0);
    pid_ = -1;
  }

  int waitForExit(std::chrono::seconds timeout) {
    if (pid_ <= 0) {
      throw std::logic_error("process is not running");
    }
    const auto deadline = Clock::now() + timeout;
    int status = 0;
    while (Clock::now() < deadline) {
      const auto waited = waitpid(pid_, &status, WNOHANG);
      if (waited == pid_) {
        pid_ = -1;
        return WIFEXITED(status) ? WEXITSTATUS(status) : 1;
      }
      if (waited == -1) {
        throw std::runtime_error("waitpid failed");
      }
      std::this_thread::sleep_for(20ms);
    }
    throw std::runtime_error("process did not exit in time");
  }

private:
  pid_t pid_ = -1;
};

class TemporaryConfig {
public:
  explicit TemporaryConfig(const std::string &prefix) {
    path_ = std::filesystem::temp_directory_path() /
            ("zms-navigation-" + std::to_string(getpid()) + ".json");
    std::ofstream output(path_);
    output << Json{
        {"version", 1},
        {"robot_prefix", prefix},
        {"odom_timeout_s", 0.7},
        {"scan_timeout_s",
         0.7}}.dump();
    if (!output) {
      throw std::runtime_error("could not write test configuration");
    }
  }
  ~TemporaryConfig() {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }
  const std::filesystem::path &path() const { return path_; }

private:
  std::filesystem::path path_;
};

class TemporaryManipulationConfig {
public:
  TemporaryManipulationConfig(const std::string &prefix,
                              const std::filesystem::path &repository,
                              double trackingError = 0.4) {
    path_ = std::filesystem::temp_directory_path() /
            ("zms-manipulation-" + std::to_string(getpid()) + ".json");
    std::ofstream output(path_);
    output
        << Json{{"version", 1},
                {"robot_prefix", prefix},
                {"urdf_path",
                 (repository / "config/examples/two_joint_arm.urdf").string()},
                {"base_frame", "base"},
                {"eef_frame", "tool"},
                {"tracking_error", trackingError}}
               .dump();
    if (!output) {
      throw std::runtime_error("could not write manipulation configuration");
    }
  }
  ~TemporaryManipulationConfig() {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }
  const std::filesystem::path &path() const { return path_; }

private:
  std::filesystem::path path_;
};

class Observations {
public:
  void result(const zenoh::Sample &sample) {
    try {
      const auto value = Json::parse(sample.get_payload().as_string());
      std::lock_guard lock(mutex_);
      if (results_.size() == 128) {
        results_.pop_front();
      }
      results_.push_back(value);
      ready_.notify_all();
    } catch (const std::exception &) {
    }
  }

  void velocity(const zenoh::Sample &sample) {
    try {
      const auto value = Json::parse(sample.get_payload().as_string());
      std::lock_guard lock(mutex_);
      lastVelocity_ = value;
      ++velocitySequence_;
      ready_.notify_all();
    } catch (const std::exception &) {
    }
  }

  void odometry(const zenoh::Sample &sample) {
    try {
      const auto value = Json::parse(sample.get_payload().as_string());
      std::lock_guard lock(mutex_);
      lastOdom_ = value;
      maximumY_ = std::max(maximumY_, value.at("y").get<double>());
      trail_.emplace_back(value.at("x").get<double>(),
                          value.at("y").get<double>());
      ready_.notify_all();
    } catch (const std::exception &) {
    }
  }

  void jointStates(const zenoh::Sample &sample) {
    try {
      const auto value = Json::parse(sample.get_payload().as_string());
      std::lock_guard lock(mutex_);
      lastJointStates_ = value;
      ready_.notify_all();
    } catch (const std::exception &) {
    }
  }

  void path(const zenoh::Sample &sample) {
    try {
      const auto value = Json::parse(sample.get_payload().as_string());
      if (!value.contains("contract") || value.at("contract") != 1 ||
          !value.contains("poses") || !value.at("poses").is_array()) {
        return;
      }
      std::lock_guard lock(mutex_);
      ++pathCount_;
      pathSizes_.push_back(value.at("poses").size());
      ready_.notify_all();
    } catch (const std::exception &) {
    }
  }

  void costmap(const zenoh::Sample &sample) {
    try {
      const auto value = Json::parse(sample.get_payload().as_string());
      if (!value.contains("width") || !value.contains("height") ||
          !value.contains("data")) {
        return;
      }
      std::lock_guard lock(mutex_);
      ++costmapCount_;
      costmapWidth_ = value.at("width").get<int>();
      costmapHeight_ = value.at("height").get<int>();
      costmapDataSize_ = value.at("data").size();
      ready_.notify_all();
    } catch (const std::exception &) {
    }
  }

  void trajectory(const zenoh::Sample &sample) {
    try {
      const auto value = Json::parse(sample.get_payload().as_string());
      if (!value.contains("contract") || value.at("contract") != 1 ||
          !value.contains("points") || !value.at("points").is_array()) {
        return;
      }
      std::lock_guard lock(mutex_);
      ++trajectoryCount_;
      trajectoryPoints_ = value.at("points").size();
      trajectoryHasEef_ = value.contains("eef_path");
      if (trajectoryHasEef_) {
        ++eefTrajectoryCount_;
      }
      ready_.notify_all();
    } catch (const std::exception &) {
    }
  }

  bool waitForPathAfter(std::size_t before, std::chrono::seconds timeout) {
    std::unique_lock lock(mutex_);
    return ready_.wait_for(lock, timeout, [&] { return pathCount_ > before; });
  }

  bool waitForCostmapSize(int width, int height, std::chrono::seconds timeout) {
    std::unique_lock lock(mutex_);
    return ready_.wait_for(lock, timeout, [&] {
      return costmapWidth_ == width && costmapHeight_ == height &&
             costmapDataSize_ ==
                 static_cast<std::size_t>(width) * height;
    });
  }

  std::size_t pathCount() {
    std::lock_guard lock(mutex_);
    return pathCount_;
  }

  bool waitForTrajectory(std::chrono::seconds timeout) {
    std::unique_lock lock(mutex_);
    return ready_.wait_for(lock, timeout, [&] { return trajectoryCount_ > 0; });
  }

  bool waitForEefTrajectory(std::chrono::seconds timeout) {
    std::unique_lock lock(mutex_);
    return ready_.wait_for(lock, timeout,
                           [&] { return eefTrajectoryCount_ > 0; });
  }

  bool trajectoryStayedBounded() {
    std::lock_guard lock(mutex_);
    return trajectoryPoints_ > 0 && trajectoryPoints_ <= 200;
  }

  bool trajectoryHasEndEffectorPath() {
    std::lock_guard lock(mutex_);
    return trajectoryHasEef_;
  }

  std::optional<Json> waitForResult(const std::string &commandId,
                                    std::chrono::seconds timeout) {
    std::unique_lock lock(mutex_);
    if (!ready_.wait_for(lock, timeout, [&] {
          return std::any_of(
              results_.begin(), results_.end(), [&](const Json &value) {
                return value.value("command_id", "") == commandId;
              });
        })) {
      return std::nullopt;
    }
    const auto found =
        std::find_if(results_.begin(), results_.end(), [&](const Json &value) {
          return value.value("command_id", "") == commandId;
        });
    Json value = *found;
    results_.erase(found);
    return value;
  }

  bool waitForOdom(std::chrono::seconds timeout) {
    std::unique_lock lock(mutex_);
    return ready_.wait_for(lock, timeout,
                           [&] { return lastOdom_.has_value(); });
  }

  bool waitForJoints(double first, double second,
                     std::chrono::milliseconds timeout) {
    std::unique_lock lock(mutex_);
    return ready_.wait_for(lock, timeout, [&] {
      if (!lastJointStates_ || !lastJointStates_->contains("position") ||
          !lastJointStates_->at("position").is_array() ||
          lastJointStates_->at("position").size() < 2) {
        return false;
      }
      return std::abs(lastJointStates_->at("position").at(0).get<double>() -
                      first) < 0.02 &&
             std::abs(lastJointStates_->at("position").at(1).get<double>() -
                      second) < 0.02;
    });
  }

  double closestApproach(double x, double y) {
    std::lock_guard lock(mutex_);
    double closest = std::numeric_limits<double>::infinity();
    for (const auto &[px, py] : trail_) {
      closest = std::min(closest, std::hypot(px - x, py - y));
    }
    return closest;
  }

  double x() {
    std::lock_guard lock(mutex_);
    return lastOdom_.value().at("x").get<double>();
  }

  double maximumY() {
    std::lock_guard lock(mutex_);
    return maximumY_;
  }

  Json latestOdom() {
    std::lock_guard lock(mutex_);
    return lastOdom_.value();
  }

  std::size_t velocitySequence() {
    std::lock_guard lock(mutex_);
    return velocitySequence_;
  }

  bool waitForZeroAfter(std::size_t sequence, std::chrono::seconds timeout) {
    std::unique_lock lock(mutex_);
    return ready_.wait_for(lock, timeout, [&] {
      return velocitySequence_ > sequence && lastVelocity_ &&
             lastVelocity_->value("linear", 1.0) == 0.0 &&
             lastVelocity_->value("angular", 1.0) == 0.0;
    });
  }

private:
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<Json> results_;
  std::optional<Json> lastVelocity_;
  std::optional<Json> lastOdom_;
  std::vector<std::pair<double, double>> trail_;
  std::optional<Json> lastJointStates_;
  double maximumY_ = 0.0;
  std::size_t velocitySequence_ = 0;
  std::size_t pathCount_ = 0;
  std::vector<std::size_t> pathSizes_;
  std::size_t costmapCount_ = 0;
  int costmapWidth_ = 0;
  int costmapHeight_ = 0;
  std::size_t costmapDataSize_ = 0;
  std::size_t trajectoryCount_ = 0;
  std::size_t trajectoryPoints_ = 0;
  std::size_t eefTrajectoryCount_ = 0;
  bool trajectoryHasEef_ = false;
};

Json requestGoal(zenoh::Session &session, const std::string &prefix,
                 const std::string &commandId, double x) {
  return zms::contract::query(
      session, zms::contract::key(prefix, "navigation/goal"),
      zms::contract::toJson(zms::contract::Goal{
          commandId, "integration-client", {{x, 0.0, 0.0}}}));
}

Json requestCancel(zenoh::Session &session, const std::string &prefix,
                   const std::string &commandId) {
  return zms::contract::query(session,
                              zms::contract::key(prefix, "navigation/cancel"),
                              zms::contract::toJson(zms::contract::Cancel{
                                  commandId, "integration-client"}));
}

void waitForStack(zenoh::Session &session, const std::string &prefix,
                  const std::string &stack = "navigation") {
  const auto deadline = Clock::now() + 5s;
  while (Clock::now() < deadline) {
    try {
      const auto response = zms::contract::query(
          session, zms::contract::key(prefix, stack + "/status"), std::nullopt,
          200ms);
      if (response.value("contract", 0) == 1) {
        return;
      }
    } catch (const std::exception &) {
    }
    std::this_thread::sleep_for(50ms);
  }
  throw std::runtime_error(stack + " stack did not become ready");
}

}

TEST(Integration, UnchangedGlobalCostmapReachesLateSubscriber) {
  const auto executableDirectory =
      std::filesystem::canonical("/proc/self/exe").parent_path();
  const std::string prefix = "late-costmap/" + std::to_string(getpid());
  TemporaryConfig config(prefix);
  Process stack(executableDirectory / "zms_navigation",
                {"--config", config.path().string()});
  zenoh::Session session(zenoh::Config::create_default());
  Observations initial;
  const auto costmapKey = zms::contract::key(prefix, "navigation/costmap");
  auto firstSubscriber = session.declare_subscriber(
      zenoh::KeyExpr(costmapKey),
      [&](const zenoh::Sample &sample) { initial.costmap(sample); }, [] {});
  waitForStack(session, prefix);
  const Json map = {{"contract", 1},
                    {"width", 20},
                    {"height", 15},
                    {"resolution", 0.1},
                    {"origin", {{"x", -1.0}, {"y", -1.0}, {"yaw", 0.0}}},
                    {"data", std::vector<int>(300, 0)}};
  bool mapAvailable = false;
  const auto deadline = Clock::now() + 5s;
  while (!mapAvailable && Clock::now() < deadline) {
    zms::contract::publish(session, zms::contract::key(prefix, "map"), map);
    const auto status = zms::contract::query(
        session, zms::contract::key(prefix, "navigation/status"), std::nullopt);
    mapAvailable = status.value("map_available", false);
    if (!mapAvailable) {
      std::this_thread::sleep_for(50ms);
    }
  }
  ASSERT_TRUE(mapAvailable);
  ASSERT_TRUE(initial.waitForCostmapSize(20, 15, 5s));
  Observations late;
  auto lateSubscriber = session.declare_subscriber(
      zenoh::KeyExpr(costmapKey),
      [&](const zenoh::Sample &sample) { late.costmap(sample); }, [] {});
  EXPECT_TRUE(late.waitForCostmapSize(20, 15, 3s));
}

TEST(Integration, NavigationWorkflow) {
  const auto executableDirectory =
      std::filesystem::canonical("/proc/self/exe").parent_path();
  const std::string prefix = "integration/" + std::to_string(getpid());
  TemporaryConfig config(prefix);
  Process mock(executableDirectory / "zms_mock_robot",
               {prefix, "--joints", "joint_1,joint_2"});
  Process stack(executableDirectory / "zms_navigation",
                {"--config", config.path().string()});
  zenoh::Session session(zenoh::Config::create_default());
  Observations observations;
  auto result = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "navigation/result")),
      [&](const zenoh::Sample &sample) { observations.result(sample); }, [] {});
  auto velocity = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "cmd_vel")),
      [&](const zenoh::Sample &sample) { observations.velocity(sample); },
      [] {});
  auto odometry = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "odom")),
      [&](const zenoh::Sample &sample) { observations.odometry(sample); },
      [] {});
  auto jointStates = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "joint_states")),
      [&](const zenoh::Sample &sample) { observations.jointStates(sample); },
      [] {});
  auto path = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "navigation/path")),
      [&](const zenoh::Sample &sample) { observations.path(sample); }, [] {});
  auto costmap = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "navigation/costmap")),
      [&](const zenoh::Sample &sample) { observations.costmap(sample); },
      [] {});

  waitForStack(session, prefix);
  ASSERT_TRUE(observations.waitForOdom(5s));
  bool jointsReached = false;
  for (int attempt = 0; attempt < 6 && !jointsReached; ++attempt) {
    Process cli(executableDirectory / "zms_cli",
                {prefix, "integration-client", "mock-joints", "joint_1,joint_2",
                 "0.3,-0.2"});
    ASSERT_EQ(cli.waitForExit(3s), 0);
    jointsReached = observations.waitForJoints(0.3, -0.2, 500ms);
  }
  ASSERT_TRUE(jointsReached);
  ASSERT_TRUE(requestGoal(session, prefix, "goal", 0.5).at("accepted"));
  const auto reached = observations.waitForResult("goal", 8s);
  ASSERT_TRUE(reached.has_value());
  EXPECT_EQ(reached->at("state"), "succeeded");

  const auto route = zms::contract::toJson(zms::contract::Goal{
      "route", "integration-client", {{0.7, 0.0, 0.0}, {0.9, 0.0, 0.0}}});
  ASSERT_TRUE(zms::contract::query(
                  session, zms::contract::key(prefix, "navigation/goal"), route)
                  .at("accepted"));
  const auto routeResult = observations.waitForResult("route", 8s);
  ASSERT_TRUE(routeResult.has_value());
  EXPECT_EQ(routeResult->at("state"), "succeeded");

  Json cells = Json::array();
  for (int y = 0; y < 15; ++y) {
    for (int x = 0; x < 20; ++x) {
      cells.push_back(x == 7 && (y < 5 || y > 13) ? 100 : 0);
    }
  }
  const Json map = {{"contract", 1},
                    {"width", 20},
                    {"height", 15},
                    {"resolution", 0.2},
                    {"origin", {{"x", 0.0}, {"y", -1.0}, {"yaw", 0.0}}},
                    {"data", cells}};
  bool mapAvailable = false;
  for (int attempt = 0; attempt < 20 && !mapAvailable; ++attempt) {
    zms::contract::publish(session, zms::contract::key(prefix, "map"), map);
    std::this_thread::sleep_for(100ms);
    const auto status = zms::contract::query(
        session, zms::contract::key(prefix, "navigation/status"), std::nullopt);
    mapAvailable = status.value("map_available", false);
  }
  ASSERT_TRUE(mapAvailable);
  const auto pathBeforeMapGoal = observations.pathCount();
  ASSERT_TRUE(requestGoal(session, prefix, "map-goal", 2.0).at("accepted"));
  EXPECT_TRUE(observations.waitForPathAfter(pathBeforeMapGoal, 5s));
  EXPECT_TRUE(observations.waitForCostmapSize(20, 15, 5s));
  const auto mapResult = observations.waitForResult("map-goal", 30s);
  if (!mapResult) {
    const auto status = zms::contract::query(
        session, zms::contract::key(prefix, "navigation/status"), std::nullopt);
    ADD_FAILURE() << "map goal timed out; status=" << status.dump()
                  << " odom=" << observations.latestOdom().dump()
                  << " maximum_y=" << observations.maximumY();
    requestCancel(session, prefix, "map-goal");
    return;
  }
  EXPECT_EQ(mapResult->at("state"), "succeeded");
  EXPECT_GT(observations.maximumY(), 0.3);

  ASSERT_TRUE(requestGoal(session, prefix, "cancel", 3.0).at("accepted"));
  const auto busy = zms::contract::query(
      session, zms::contract::key(prefix, "navigation/goal"),
      zms::contract::toJson(
          zms::contract::Goal{"busy", "another-client", {{2.5, 0.0, 0.0}}}));
  EXPECT_FALSE(busy.at("accepted"));
  EXPECT_EQ(busy.at("reason"), "busy");
  ASSERT_TRUE(requestCancel(session, prefix, "cancel").at("accepted"));
  const auto cancelled = observations.waitForResult("cancel", 3s);
  ASSERT_TRUE(cancelled.has_value());
  EXPECT_EQ(cancelled->at("state"), "cancelled");

  ASSERT_TRUE(requestGoal(session, prefix, "stale", 3.0).at("accepted"));
  const auto beforeStale = observations.velocitySequence();
  mock.stop();
  const auto stale = observations.waitForResult("stale", 3s);
  ASSERT_TRUE(stale.has_value());
  EXPECT_EQ(stale->at("reason"), "stale_odom");
  EXPECT_TRUE(observations.waitForZeroAfter(beforeStale, 2s));

  mock.start(executableDirectory / "zms_mock_robot",
             {prefix, "--joints", "joint_1,joint_2"});
  const auto scanKey = zms::contract::key(prefix, "scan");
  std::jthread scanPublisher([&](std::stop_token stop) {
    while (!stop.stop_requested()) {
      zms::contract::publish(session, scanKey,
                             {{"contract", 1},
                              {"angle_min", 0.0},
                              {"angle_increment", 1.0},
                              {"range_min", 0.01},
                              {"range_max", 10.0},
                              {"ranges", Json::array({0.1})}});
      std::this_thread::sleep_for(100ms);
    }
  });
  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(requestGoal(session, prefix, "obstacle", 2.0).at("accepted"));
  const auto beforeObstacle = observations.velocitySequence();
  const auto obstacleStartX = observations.x();
  EXPECT_TRUE(observations.waitForZeroAfter(beforeObstacle + 5, 2s));
  EXPECT_NEAR(observations.x(), obstacleStartX, 0.02);
  ASSERT_TRUE(requestCancel(session, prefix, "obstacle").at("accepted"));
  scanPublisher.request_stop();
  scanPublisher.join();

  ASSERT_TRUE(requestGoal(session, prefix, "shutdown", 3.0).at("accepted"));
  const auto beforeShutdown = observations.velocitySequence();
  stack.stop();
  const auto shutdown = observations.waitForResult("shutdown", 3s);
  ASSERT_TRUE(shutdown.has_value());
  EXPECT_EQ(shutdown->at("reason"), "shutdown");
  EXPECT_TRUE(observations.waitForZeroAfter(beforeShutdown, 2s));
}

TEST(Integration, NavigationAvoidsObstacleOnStraightRoute) {
  const auto executableDirectory =
      std::filesystem::canonical("/proc/self/exe").parent_path();
  const std::string prefix = "avoid-integration/" + std::to_string(getpid());
  TemporaryConfig config(prefix);
  Process mock(executableDirectory / "zms_mock_robot",
               {prefix, "--obstacle", "0.9,0.0,0.12"});
  Process stack(executableDirectory / "zms_navigation",
                {"--config", config.path().string()});
  zenoh::Session session(zenoh::Config::create_default());
  Observations observations;
  auto results = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "navigation/result")),
      [&](const zenoh::Sample &sample) { observations.result(sample); }, [] {});
  auto odometry = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "odom")),
      [&](const zenoh::Sample &sample) { observations.odometry(sample); },
      [] {});
  waitForStack(session, prefix, "navigation");
  ASSERT_TRUE(observations.waitForOdom(5s));
  std::this_thread::sleep_for(500ms);
  ASSERT_TRUE(requestGoal(session, prefix, "around", 1.8).at("accepted"));
  const auto result = observations.waitForResult("around", 40s);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->at("state"), "succeeded") << result->dump();
  EXPECT_GT(observations.closestApproach(0.9, 0.0), 0.12 + 0.25);
}

TEST(Integration, NavigationLocalizesWithDriftingOdometry) {
  const auto executableDirectory =
      std::filesystem::canonical("/proc/self/exe").parent_path();
  const std::string prefix = "localize-integration/" + std::to_string(getpid());
  const auto configPath =
      std::filesystem::temp_directory_path() /
      ("zms-localize-" + std::to_string(getpid()) + ".json");
  {
    std::ofstream output(configPath);
    output << Json{{"version", 1},
                   {"robot_prefix", prefix},
                   {"odom_timeout_s", 0.7},
                   {"scan_timeout_s", 0.7},
                   {"progress_timeout_s", 30.0},
                   {"localization",
                    {{"enabled", true},
                     {"initial_pose", {{"x", 1.0}, {"y", 1.0}, {"yaw", 0.0}}}}}}
                  .dump();
  }
  constexpr double width = 6.0;
  constexpr double height = 4.0;
  constexpr double resolution = 0.05;
  const int cellsX = static_cast<int>(width / resolution);
  const int cellsY = static_cast<int>(height / resolution);
  Json data = Json::array();
  for (int y = 0; y < cellsY; ++y) {
    for (int x = 0; x < cellsX; ++x) {
      const bool border =
          x == 0 || y == 0 || x == cellsX - 1 || y == cellsY - 1;
      const bool box = x >= 60 && x < 70 && y >= 50 && y < 60;
      data.push_back(border || box ? 100 : 0);
    }
  }
  const Json map = {{"contract", 1},
                    {"width", cellsX},
                    {"height", cellsY},
                    {"resolution", resolution},
                    {"origin", {{"x", 0.0}, {"y", 0.0}, {"yaw", 0.0}}},
                    {"data", data}};
  Process mock(executableDirectory / "zms_mock_robot", {prefix,
                                                        "--start",
                                                        "1.0,1.0,0.0",
                                                        "--odom-scale",
                                                        "1.1",
                                                        "--odom-yaw-drift",
                                                        "0.03",
                                                        "--wall",
                                                        "0.05,0.05,5.95,0.05",
                                                        "--wall",
                                                        "5.95,0.05,5.95,3.95",
                                                        "--wall",
                                                        "5.95,3.95,0.05,3.95",
                                                        "--wall",
                                                        "0.05,3.95,0.05,0.05",
                                                        "--wall",
                                                        "3.0,2.5,3.5,2.5",
                                                        "--wall",
                                                        "3.5,2.5,3.5,3.0",
                                                        "--wall",
                                                        "3.5,3.0,3.0,3.0",
                                                        "--wall",
                                                        "3.0,3.0,3.0,2.5"});
  Process stack(executableDirectory / "zms_navigation",
                {"--config", configPath.string()});
  zenoh::Session session(zenoh::Config::create_default());
  Observations observations;
  Observations truth;
  auto results = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "navigation/result")),
      [&](const zenoh::Sample &sample) { observations.result(sample); }, [] {});
  auto truthSubscriber = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "ground_truth")),
      [&](const zenoh::Sample &sample) { truth.odometry(sample); }, [] {});
  waitForStack(session, prefix, "navigation");
  ASSERT_TRUE(truth.waitForOdom(5s));
  std::jthread mapPublisher([&](std::stop_token stop) {
    while (!stop.stop_requested()) {
      zms::contract::publish(session, zms::contract::key(prefix, "map"), map);
      std::this_thread::sleep_for(500ms);
    }
  });
  std::this_thread::sleep_for(1500ms);
  const auto accepted = zms::contract::query(
      session, zms::contract::key(prefix, "navigation/goal"),
      zms::contract::toJson(
          zms::contract::Goal{"localized",
                              "integration-client",
                              {{4.5, 1.5, 0.0}, {4.5, 3.2, 0.0}}}));
  ASSERT_TRUE(accepted.at("accepted")) << accepted.dump();
  const auto result = observations.waitForResult("localized", 90s);
  mapPublisher.request_stop();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->at("state"), "succeeded") << result->dump();
  const auto finalPose = truth.latestOdom();
  EXPECT_NEAR(finalPose.at("x").get<double>(), 4.5, 0.3);
  EXPECT_NEAR(finalPose.at("y").get<double>(), 3.2, 0.3);
  std::filesystem::remove(configPath);
}

TEST(Integration, ManipulationJointWorkflow) {
  const auto executableDirectory =
      std::filesystem::canonical("/proc/self/exe").parent_path();
  const auto repository = executableDirectory.parent_path().parent_path();
  const std::string prefix =
      "manipulation-integration/" + std::to_string(getpid());
  TemporaryManipulationConfig config(prefix, repository);
  Process mock(executableDirectory / "zms_mock_robot",
               {prefix, "--joints", "joint_1,joint_2,gripper"});
  Process stack(executableDirectory / "zms_manipulation",
                {"--config", config.path().string()});
  zenoh::Session session(zenoh::Config::create_default());
  Observations observations;
  auto results = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "manipulation/result")),
      [&](const zenoh::Sample &sample) { observations.result(sample); }, [] {});
  auto states = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "joint_states")),
      [&](const zenoh::Sample &sample) { observations.jointStates(sample); },
      [] {});
  auto trajectory = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "manipulation/trajectory")),
      [&](const zenoh::Sample &sample) { observations.trajectory(sample); },
      [] {});
  waitForStack(session, prefix, "manipulation");
  ASSERT_TRUE(observations.waitForJoints(0.0, 0.0, 5s));
  const auto targetKey = zms::contract::key(prefix, "manipulation/target");
  const auto sendTarget = [&](const std::string &commandId,
                              const std::string &clientId, double first,
                              double second) {
    return zms::contract::query(
        session, targetKey,
        zms::contract::toJson(zms::contract::Target{
            commandId, clientId,
            zms::contract::JointTarget{{"joint_1", "joint_2"},
                                       {first, second}}}));
  };
  Json response;
  for (int attempt = 0; attempt < 10; ++attempt) {
    response = sendTarget("joint-goal", "owner", 0.5, -0.4);
    if (response.value("accepted", false)) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
  ASSERT_TRUE(response.at("accepted")) << response.dump();
  EXPECT_TRUE(observations.waitForTrajectory(5s));
  EXPECT_TRUE(observations.trajectoryStayedBounded());
  EXPECT_FALSE(observations.trajectoryHasEndEffectorPath());
  const auto reached = observations.waitForResult("joint-goal", 5s);
  ASSERT_TRUE(reached.has_value());
  EXPECT_EQ(reached->at("state"), "succeeded");
  EXPECT_TRUE(observations.waitForJoints(0.5, -0.4, 1s));

  const auto outside = sendTarget("outside", "owner", 2.0, 0.0);
  EXPECT_FALSE(outside.at("accepted"));
  EXPECT_EQ(outside.at("reason"), "invalid_target");

  const auto pose =
      zms::contract::query(session, targetKey,
                           zms::contract::toJson(zms::contract::Target{
                               "pose", "owner", zms::contract::PoseTarget{}}));
  EXPECT_FALSE(pose.at("accepted"));
  EXPECT_EQ(pose.at("reason"), "unreachable");
  const auto noGripper = zms::contract::query(
      session, zms::contract::key(prefix, "manipulation/gripper"),
      zms::contract::toJson(
          zms::contract::GripperCommand{"grip", "owner", "close", {}}));
  EXPECT_FALSE(noGripper.at("accepted"));
  EXPECT_EQ(noGripper.at("reason"), "unsupported");

  zms::contract::PoseTarget reachable;
  reachable.x = 0.3 * std::cos(0.4);
  reachable.y = 0.3 * std::sin(0.4);
  reachable.z = 0.2;
  reachable.qz = std::sin(0.05);
  reachable.qw = std::cos(0.05);
  const auto posed =
      zms::contract::query(session, targetKey,
                           zms::contract::toJson(zms::contract::Target{
                               "pose-goal", "owner", reachable}));
  ASSERT_TRUE(posed.at("accepted")) << posed.dump();
  EXPECT_TRUE(observations.waitForEefTrajectory(5s));
  const auto posedResult = observations.waitForResult("pose-goal", 5s);
  ASSERT_TRUE(posedResult.has_value());
  EXPECT_EQ(posedResult->at("state"), "succeeded");
  EXPECT_TRUE(observations.waitForJoints(0.4, -0.3, 1s));

  ASSERT_TRUE(sendTarget("cancel-goal", "owner", -1.5, 1.5).at("accepted"));
  const auto busy = sendTarget("busy-goal", "other-client", 0.0, 0.0);
  EXPECT_FALSE(busy.at("accepted"));
  EXPECT_EQ(busy.at("reason"), "busy");
  const auto cancel = zms::contract::query(
      session, zms::contract::key(prefix, "manipulation/cancel"),
      zms::contract::toJson(zms::contract::Cancel{"cancel-goal", "owner"}));
  ASSERT_TRUE(cancel.at("accepted"));
  const auto cancelled = observations.waitForResult("cancel-goal", 3s);
  ASSERT_TRUE(cancelled.has_value());
  EXPECT_EQ(cancelled->at("state"), "cancelled");

  ASSERT_TRUE(sendTarget("stale-goal", "owner", 1.5, -1.5).at("accepted"));
  mock.stop();
  const auto stale = observations.waitForResult("stale-goal", 3s);
  ASSERT_TRUE(stale.has_value());
  EXPECT_EQ(stale->at("reason"), "stale_joint_states");
  mock.start(executableDirectory / "zms_mock_robot",
             {prefix, "--joints", "joint_1,joint_2"});
  ASSERT_TRUE(observations.waitForJoints(0.0, 0.0, 3s));

  ASSERT_TRUE(sendTarget("shutdown-goal", "owner", 1.5, -1.5).at("accepted"));
  stack.stop();
  const auto shutdown = observations.waitForResult("shutdown-goal", 3s);
  ASSERT_TRUE(shutdown.has_value());
  EXPECT_EQ(shutdown->at("reason"), "shutdown");
}

TEST(Integration, GripperOpensAndCloses) {
  const auto executableDirectory =
      std::filesystem::canonical("/proc/self/exe").parent_path();
  const auto repository = executableDirectory.parent_path().parent_path();
  const std::string prefix = "gripper-integration/" + std::to_string(getpid());
  const auto configPath = std::filesystem::temp_directory_path() /
                          ("zms-gripper-" + std::to_string(getpid()) + ".json");
  {
    std::ofstream output(configPath);
    output << Json{{"version", 1},
                   {"robot_prefix", prefix},
                   {"urdf_path",
                    (repository / "config/examples/mobile_manipulator.urdf")
                        .string()},
                   {"base_frame", "base_link"},
                   {"eef_frame", "tool_flange"},
                   {"gripper",
                    {{"joints", {"left_finger_joint", "right_finger_joint"}},
                     {"open", {0.035, 0.035}},
                     {"closed", {0.0, 0.0}}}}}
                  .dump();
  }
  Process mock(executableDirectory / "zms_mock_robot",
               {prefix, "--joints",
                "arm_base_joint,upper_arm_joint,forearm_joint,wrist_roll_joint,"
                "wrist_pitch_joint,tool_flange_joint,left_finger_joint,"
                "right_finger_joint"});
  Process stack(executableDirectory / "zms_manipulation",
                {"--config", configPath.string()});
  zenoh::Session session(zenoh::Config::create_default());
  Observations observations;
  auto results = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "manipulation/result")),
      [&](const zenoh::Sample &sample) { observations.result(sample); }, [] {});
  waitForStack(session, prefix, "manipulation");
  std::this_thread::sleep_for(1s);
  const auto gripperKey = zms::contract::key(prefix, "manipulation/gripper");
  const auto send = [&](const std::string &id, const std::string &client,
                        const std::string &action) {
    return zms::contract::query(
        session, gripperKey,
        zms::contract::toJson(
            zms::contract::GripperCommand{id, client, action, {}}));
  };
  ASSERT_TRUE(send("open", "owner", "open").at("accepted"));
  EXPECT_FALSE(send("steal", "other", "close").at("accepted"));
  const auto opened = observations.waitForResult("open", 10s);
  ASSERT_TRUE(opened.has_value());
  EXPECT_EQ(opened->at("state"), "succeeded") << opened->dump();
  ASSERT_TRUE(send("close", "owner", "close").at("accepted"));
  const auto closed = observations.waitForResult("close", 10s);
  ASSERT_TRUE(closed.has_value());
  EXPECT_EQ(closed->at("state"), "succeeded") << closed->dump();
  std::filesystem::remove(configPath);
}

TEST(Integration, ManipulationTrackingFailure) {
  const auto executableDirectory =
      std::filesystem::canonical("/proc/self/exe").parent_path();
  const auto repository = executableDirectory.parent_path().parent_path();
  const std::string prefix = "tracking-integration/" + std::to_string(getpid());
  TemporaryManipulationConfig config(prefix, repository, 0.05);
  Process stack(executableDirectory / "zms_manipulation",
                {"--config", config.path().string()});
  zenoh::Session session(zenoh::Config::create_default());
  Observations observations;
  auto result = session.declare_subscriber(
      zenoh::KeyExpr(zms::contract::key(prefix, "manipulation/result")),
      [&](const zenoh::Sample &sample) { observations.result(sample); }, [] {});
  const auto stateKey = zms::contract::key(prefix, "joint_states");
  std::jthread states([&](std::stop_token stop) {
    while (!stop.stop_requested()) {
      zms::contract::publish(session, stateKey,
                             {{"contract", 1},
                              {"name", Json::array({"joint_1", "joint_2"})},
                              {"position", Json::array({0.0, 0.0})},
                              {"velocity", Json::array({0.0, 0.0})}});
      std::this_thread::sleep_for(20ms);
    }
  });
  waitForStack(session, prefix, "manipulation");
  Json response;
  for (int attempt = 0; attempt < 10; ++attempt) {
    response = zms::contract::query(
        session, zms::contract::key(prefix, "manipulation/target"),
        zms::contract::toJson(zms::contract::Target{
            "tracking-goal", "owner",
            zms::contract::JointTarget{{"joint_1", "joint_2"}, {0.5, 0.0}}}));
    if (response.value("accepted", false)) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
  ASSERT_TRUE(response.at("accepted")) << response.dump();
  const auto failure = observations.waitForResult("tracking-goal", 3s);
  ASSERT_TRUE(failure.has_value());
  EXPECT_EQ(failure->at("state"), "failed");
  EXPECT_EQ(failure->at("reason"), "tracking_error");
}
