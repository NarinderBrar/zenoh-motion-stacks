#include <zms/contract/Contract.hpp>
#include <zms/contract/ZenohClient.hpp>

#include <zenoh.hxx>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Json = nlohmann::json;
using zms::contract::query;

double number(const char *text) {
  std::size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != std::string(text).size() || !std::isfinite(value)) {
    throw std::invalid_argument("pose values must be finite numbers");
  }
  return value;
}

int seconds(const char *text) {
  std::size_t consumed = 0;
  const int value = std::stoi(text, &consumed);
  if (consumed != std::string(text).size() || value < 1 || value > 3600) {
    throw std::invalid_argument("watch timeout must be 1 to 3600 seconds");
  }
  return value;
}

std::vector<std::string> split(const std::string &value) {
  std::vector<std::string> items;
  std::size_t start = 0;
  while (start < value.size()) {
    const auto end = value.find(',', start);
    const auto item = value.substr(start, end - start);
    if (item.empty()) {
      throw std::invalid_argument("comma-separated items must be nonempty");
    }
    items.push_back(item);
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  if (items.empty() || value.back() == ',') {
    throw std::invalid_argument("comma-separated items must be nonempty");
  }
  return items;
}

zms::contract::JointTarget jointsFromArguments(const char *names,
                                               const char *positions) {
  zms::contract::JointTarget target;
  target.names = split(names);
  for (const auto &position : split(positions)) {
    target.positions.push_back(number(position.c_str()));
  }
  return target;
}

Json watchResult(zenoh::Session &session, const std::string &prefix,
                 const std::string &stack, const std::string &commandId,
                 int timeoutSeconds) {
  if ((stack != "navigation" && stack != "manipulation") ||
      timeoutSeconds < 1 || timeoutSeconds > 3600) {
    throw std::invalid_argument("invalid stack or watch timeout");
  }
  struct State {
    std::mutex mutex;
    std::condition_variable ready;
    std::optional<Json> result;
  };
  auto state = std::make_shared<State>();
  const auto feedbackKey = zms::contract::key(prefix, stack + "/feedback");
  const auto resultKey = zms::contract::key(prefix, stack + "/result");
  auto feedback = session.declare_subscriber(
      zenoh::KeyExpr(feedbackKey),
      [state, commandId](const zenoh::Sample &sample) {
        try {
          const auto value = Json::parse(sample.get_payload().as_string());
          if (value.value("command_id", "") == commandId) {
            std::lock_guard lock(state->mutex);
            std::cout << value.dump() << '\n';
          }
        } catch (const std::exception &) {
        }
      },
      [] {});
  auto result = session.declare_subscriber(
      zenoh::KeyExpr(resultKey),
      [state, commandId](const zenoh::Sample &sample) {
        try {
          auto value = Json::parse(sample.get_payload().as_string());
          if (value.value("command_id", "") == commandId) {
            {
              std::lock_guard lock(state->mutex);
              state->result = std::move(value);
            }
            state->ready.notify_one();
          }
        } catch (const std::exception &) {
        }
      },
      [] {});
  std::unique_lock lock(state->mutex);
  if (!state->ready.wait_for(lock, std::chrono::seconds(timeoutSeconds),
                             [&] { return state->result.has_value(); })) {
    throw std::runtime_error("result watch timed out");
  }
  return *state->result;
}

}

int main(int argc, char **argv) {
  try {
    if (argc < 4) {
      throw std::invalid_argument(
          "usage: zms_cli <robot-prefix> <client-id> goal <command-id> <x> <y> "
          "<yaw> | cancel <command-id> | status | joint-target <command-id> "
          "<names> <positions> | pose-target <command-id> <x> <y> <z> "
          "<qx> <qy> <qz> <qw> | cartesian-target <command-id> <x> <y> <z> "
          "<qx> <qy> <qz> <qw> | gripper <command-id> open|close|<positions> | "
          "cancel-manipulation <command-id> | "
          "manipulation-status | mock-joints <names> <positions> | watch "
          "<navigation|manipulation> <command-id> <timeout-s>");
    }
    const std::string prefix = argv[1];
    const std::string clientId = argv[2];
    const std::string command = argv[3];
    zenoh::Session session(zenoh::Config::create_default());
    Json response;
    if (command == "goal" && argc == 8) {
      const Json request = zms::contract::toJson(zms::contract::Goal{
          argv[4],
          clientId,
          {{number(argv[5]), number(argv[6]), number(argv[7])}}});
      response = query(session, zms::contract::key(prefix, "navigation/goal"),
                       request);
    } else if (command == "cancel" && argc == 5) {
      const Json request =
          zms::contract::toJson(zms::contract::Cancel{argv[4], clientId});
      response = query(session, zms::contract::key(prefix, "navigation/cancel"),
                       request);
    } else if (command == "status" && argc == 4) {
      response = query(session, zms::contract::key(prefix, "navigation/status"),
                       std::nullopt);
    } else if (command == "joint-target" && argc == 7) {
      const auto target = zms::contract::Target{
          argv[4], clientId, jointsFromArguments(argv[5], argv[6])};
      response =
          query(session, zms::contract::key(prefix, "manipulation/target"),
                zms::contract::toJson(target));
    } else if (command == "pose-target" && argc == 12) {
      const auto target = zms::contract::Target{
          argv[4], clientId,
          zms::contract::PoseTarget{number(argv[5]), number(argv[6]),
                                    number(argv[7]), number(argv[8]),
                                    number(argv[9]), number(argv[10]),
                                    number(argv[11])}};
      response =
          query(session, zms::contract::key(prefix, "manipulation/target"),
                zms::contract::toJson(target));
    } else if (command == "cartesian-target" && argc == 12) {
      auto pose = zms::contract::PoseTarget{
          number(argv[5]), number(argv[6]),  number(argv[7]), number(argv[8]),
          number(argv[9]), number(argv[10]), number(argv[11])};
      pose.cartesian = true;
      response =
          query(session, zms::contract::key(prefix, "manipulation/target"),
                zms::contract::toJson(
                    zms::contract::Target{argv[4], clientId, pose}));
    } else if (command == "gripper" && argc == 6) {
      zms::contract::GripperCommand gripper{argv[4], clientId, argv[5], {}};
      if (gripper.action != "open" && gripper.action != "close") {
        gripper.action = "position";
        for (const auto &position : split(argv[5])) {
          gripper.positions.push_back(number(position.c_str()));
        }
      }
      response =
          query(session, zms::contract::key(prefix, "manipulation/gripper"),
                zms::contract::toJson(gripper));
    } else if (command == "cancel-manipulation" && argc == 5) {
      response = query(
          session, zms::contract::key(prefix, "manipulation/cancel"),
          zms::contract::toJson(zms::contract::Cancel{argv[4], clientId}));
    } else if (command == "manipulation-status" && argc == 4) {
      response =
          query(session, zms::contract::key(prefix, "manipulation/status"),
                std::nullopt);
    } else if (command == "mock-joints" && argc == 6) {
      const auto joints = jointsFromArguments(argv[4], argv[5]);
      zms::contract::toJson(
          zms::contract::Target{"validation", clientId, joints});
      zms::contract::publish(session, zms::contract::key(prefix, "cmd_joints"),
                             Json{{"contract", 1},
                                  {"name", joints.names},
                                  {"position", joints.positions}});
      response = {{"contract", 1}, {"sent", true}};
    } else if (command == "watch" && argc == 7) {
      response =
          watchResult(session, prefix, argv[4], argv[5], seconds(argv[6]));
    } else {
      throw std::invalid_argument("invalid command or argument count");
    }
    std::cout << response.dump(2) << '\n';
    return response.value("accepted", true) ? 0 : 2;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
