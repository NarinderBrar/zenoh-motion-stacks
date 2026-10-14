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
    output << Json{{"version", 1},
                   {"robot_prefix", prefix},
                   {"odom_timeout_s", 0.7},
                   {"scan_timeout_s", 0.7}}
                  .dump();
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
          lastJointStates_->at("position").size() != 2) {
        return false;
      }
      return std::abs(lastJointStates_->at("position").at(0).get<double>() -
                      first) < 0.02 &&
             std::abs(lastJointStates_->at("position").at(1).get<double>() -
                      second) < 0.02;
    });
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
  std::optional<Json> lastJointStates_;
  double maximumY_ = 0.0;
  std::size_t velocitySequence_ = 0;
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

void waitForStack(zenoh::Session &session, const std::string &prefix) {
  const auto deadline = Clock::now() + 5s;
  while (Clock::now() < deadline) {
    try {
      const auto response = zms::contract::query(
          session, zms::contract::key(prefix, "navigation/status"),
          std::nullopt, 200ms);
      if (response.value("contract", 0) == 1) {
        return;
      }
    } catch (const std::exception &) {
    }
    std::this_thread::sleep_for(50ms);
  }
  throw std::runtime_error("navigation stack did not become ready");
}

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
  ASSERT_TRUE(requestGoal(session, prefix, "map-goal", 2.0).at("accepted"));
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
