#include <zms/contract/Contract.hpp>

#include <zenoh.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <mutex>
#include <numbers>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t stopRequested = 0;

void requestStop(int) { stopRequested = 1; }

struct State {
  std::mutex mutex;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double linear = 0.0;
  double angular = 0.0;
  std::vector<std::string> jointNames;
  std::vector<double> jointPositions;
  std::vector<double> jointTargets;
  std::vector<double> jointVelocities;
  std::chrono::steady_clock::time_point lastCommand =
      std::chrono::steady_clock::now();
};

std::vector<std::string> parseJointNames(const std::string &value) {
  std::vector<std::string> names;
  std::set<std::string> seen;
  std::size_t start = 0;
  while (start < value.size()) {
    const auto end = value.find(',', start);
    const auto name = value.substr(start, end - start);
    if (name.empty() || !seen.insert(name).second || names.size() >= 64) {
      throw std::invalid_argument("joint names must be unique and nonempty");
    }
    names.push_back(name);
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  if (names.empty() || value.back() == ',') {
    throw std::invalid_argument("at least one joint name is required");
  }
  return names;
}

}

int main(int argc, char **argv) {
  try {
    if (argc != 2 && argc != 4) {
      throw std::invalid_argument(
          "usage: zms_mock_robot <robot-prefix> [--joints name1,name2,...]");
    }
    if (argc == 4 && std::string_view(argv[2]) != "--joints") {
      throw std::invalid_argument("expected --joints");
    }
    const std::string prefix = argv[1];
    const auto velocityKey = zms::contract::key(prefix, "cmd_vel");
    const auto odomKey = zms::contract::key(prefix, "odom");
    const auto jointCommandKey = zms::contract::key(prefix, "cmd_joints");
    const auto jointStateKey = zms::contract::key(prefix, "joint_states");
    zenoh::Session session(zenoh::Config::create_default());
    State state;
    if (argc == 4) {
      state.jointNames = parseJointNames(argv[3]);
      state.jointPositions.resize(state.jointNames.size());
      state.jointTargets.resize(state.jointNames.size());
      state.jointVelocities.resize(state.jointNames.size());
    }
    auto subscriber = session.declare_subscriber(
        zenoh::KeyExpr(velocityKey),
        [&](const zenoh::Sample &sample) {
          try {
            if (sample.get_payload().size() > 4096) {
              return;
            }
            const auto command =
                nlohmann::json::parse(sample.get_payload().as_string());
            const double linear = command.at("linear").get<double>();
            const double angular = command.at("angular").get<double>();
            if (!std::isfinite(linear) || !std::isfinite(angular)) {
              return;
            }
            std::lock_guard lock(state.mutex);
            state.linear = std::clamp(linear, -2.0, 2.0);
            state.angular = std::clamp(angular, -4.0, 4.0);
            state.lastCommand = std::chrono::steady_clock::now();
          } catch (const std::exception &) {
          }
        },
        [] {});
    auto jointSubscriber = session.declare_subscriber(
        zenoh::KeyExpr(jointCommandKey),
        [&](const zenoh::Sample &sample) {
          try {
            if (sample.get_payload().size() > 65536) {
              return;
            }
            const auto command =
                nlohmann::json::parse(sample.get_payload().as_string());
            if (!command.contains("name") || !command.at("name").is_array() ||
                !command.contains("position") ||
                !command.at("position").is_array() ||
                command.at("name").size() != command.at("position").size()) {
              return;
            }
            std::lock_guard lock(state.mutex);
            std::vector<std::pair<std::size_t, double>> updates;
            std::set<std::string> seen;
            for (std::size_t index = 0; index < command.at("name").size();
                 ++index) {
              const auto name = command.at("name").at(index).get<std::string>();
              const double position =
                  command.at("position").at(index).get<double>();
              if (!std::isfinite(position) || !seen.insert(name).second) {
                return;
              }
              const auto found = std::find(state.jointNames.begin(),
                                           state.jointNames.end(), name);
              if (found == state.jointNames.end()) {
                return;
              }
              updates.emplace_back(
                  std::distance(state.jointNames.begin(), found), position);
            }
            for (const auto &[index, position] : updates) {
              state.jointTargets[index] = position;
            }
          } catch (const std::exception &) {
          }
        },
        [] {});
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    auto previous = std::chrono::steady_clock::now();
    while (!stopRequested) {
      const auto now = std::chrono::steady_clock::now();
      const double dt = std::chrono::duration<double>(now - previous).count();
      previous = now;
      nlohmann::json odom;
      nlohmann::json jointStates;
      {
        std::lock_guard lock(state.mutex);
        if (std::chrono::duration<double>(now - state.lastCommand).count() >
            0.5) {
          state.linear = 0.0;
          state.angular = 0.0;
        }
        state.x += std::cos(state.yaw) * state.linear * dt;
        state.y += std::sin(state.yaw) * state.linear * dt;
        state.yaw = std::remainder(state.yaw + state.angular * dt,
                                   2.0 * std::numbers::pi);
        odom = {{"contract", 1},          {"x", state.x},
                {"y", state.y},           {"yaw", state.yaw},
                {"linear", state.linear}, {"angular", state.angular}};
        for (std::size_t index = 0; index < state.jointNames.size(); ++index) {
          const double delta =
              state.jointTargets[index] - state.jointPositions[index];
          const double movement = std::clamp(delta, -dt, dt);
          state.jointPositions[index] += movement;
          state.jointVelocities[index] = dt > 0.0 ? movement / dt : 0.0;
        }
        jointStates = {{"contract", 1},
                       {"name", state.jointNames},
                       {"position", state.jointPositions},
                       {"velocity", state.jointVelocities}};
      }
      auto options = zenoh::Session::PutOptions::create_default();
      options.encoding = zenoh::Encoding("application/json");
      session.put(zenoh::KeyExpr(odomKey), zenoh::Bytes(odom.dump()),
                  std::move(options));
      if (!state.jointNames.empty()) {
        auto jointOptions = zenoh::Session::PutOptions::create_default();
        jointOptions.encoding = zenoh::Encoding("application/json");
        session.put(zenoh::KeyExpr(jointStateKey),
                    zenoh::Bytes(jointStates.dump()), std::move(jointOptions));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
