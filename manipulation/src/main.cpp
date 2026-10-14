#include <zms/contract/Contract.hpp>
#include <zms/contract/ZenohClient.hpp>
#include <zms/manipulation/JointModel.hpp>
#include <zms/manipulation/JointMotion.hpp>

#include <zenoh.hxx>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using Json = nlohmann::json;

volatile std::sig_atomic_t stopRequested = 0;

void requestStop(int) { stopRequested = 1; }

struct Config {
  std::string robotPrefix;
  std::string urdfPath;
  std::string baseFrame;
  std::string endEffectorFrame;
  int controlRateHz = 20;
  double stateTimeoutSeconds = 1.0;
  double trackingError = 0.4;
  double positionTolerance = 0.02;
  double acceleration = 2.0;
};

double positive(const Json &document, const char *field, double fallback) {
  if (!document.contains(field)) {
    return fallback;
  }
  if (!document.at(field).is_number()) {
    throw std::invalid_argument(std::string("config.") + field +
                                " must be a number");
  }
  const double value = document.at(field).get<double>();
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string("config.") + field +
                                " must be positive and finite");
  }
  return value;
}

std::string requiredString(const Json &document, const char *field) {
  if (!document.contains(field) || !document.at(field).is_string() ||
      document.at(field).get<std::string>().empty()) {
    throw std::invalid_argument(std::string("config.") + field +
                                " must be a nonempty string");
  }
  return document.at(field).get<std::string>();
}

Config loadConfig(const std::string &path) {
  std::ifstream input(path);
  if (!input) {
    throw std::invalid_argument("cannot read configuration: " + path);
  }
  const Json document = Json::parse(input);
  if (!document.is_object()) {
    throw std::invalid_argument("config must be an object");
  }
  const std::unordered_set<std::string> fields = {
      "version",         "robot_prefix",   "urdf_path",
      "base_frame",      "eef_frame",      "control_rate_hz",
      "state_timeout_s", "tracking_error", "position_tolerance",
      "max_acceleration"};
  for (auto item = document.begin(); item != document.end(); ++item) {
    if (!fields.contains(item.key())) {
      throw std::invalid_argument("config." + item.key() + " is unknown");
    }
  }
  if (!document.contains("version") || document.at("version") != 1) {
    throw std::invalid_argument("config.version must be 1");
  }
  Config result;
  if (!document.contains("robot_prefix") ||
      !document.at("robot_prefix").is_string()) {
    throw std::invalid_argument("config.robot_prefix must be a string");
  }
  result.robotPrefix = document.at("robot_prefix").get<std::string>();
  zms::contract::key(result.robotPrefix, "joint_states");
  result.urdfPath = requiredString(document, "urdf_path");
  if (std::filesystem::path(result.urdfPath).is_relative()) {
    result.urdfPath =
        (std::filesystem::absolute(path).parent_path() / result.urdfPath)
            .lexically_normal()
            .string();
  }
  result.baseFrame = requiredString(document, "base_frame");
  result.endEffectorFrame = requiredString(document, "eef_frame");
  if (document.contains("control_rate_hz")) {
    if (!document.at("control_rate_hz").is_number_integer()) {
      throw std::invalid_argument("config.control_rate_hz must be an integer");
    }
    result.controlRateHz = document.at("control_rate_hz").get<int>();
    if (result.controlRateHz < 1 || result.controlRateHz > 200) {
      throw std::invalid_argument("config.control_rate_hz must be 1 to 200");
    }
  }
  result.stateTimeoutSeconds =
      positive(document, "state_timeout_s", result.stateTimeoutSeconds);
  result.trackingError =
      positive(document, "tracking_error", result.trackingError);
  result.positionTolerance =
      positive(document, "position_tolerance", result.positionTolerance);
  result.acceleration =
      positive(document, "max_acceleration", result.acceleration);
  return result;
}

enum class Claim { Waiting, Handled, Abandoned };

struct Request {
  enum class Kind { Target, Cancel } kind;
  Json payload;
  std::promise<Json> reply;
  std::atomic<Claim> claim = Claim::Waiting;
};

struct Mailbox {
  std::mutex mutex;
  std::deque<std::shared_ptr<Request>> requests;
  std::optional<std::pair<std::vector<double>, Clock::time_point>> state;
  Json status = {{"contract", 1}, {"state", "idle"}};
};

std::optional<std::vector<double>>
orderedState(const Json &payload, const zms::manipulation::JointModel &model) {
  if (!payload.is_object() || !payload.contains("name") ||
      !payload.contains("position") || !payload.at("name").is_array() ||
      !payload.at("position").is_array() ||
      payload.at("name").size() != payload.at("position").size() ||
      payload.at("name").size() > 1024) {
    return std::nullopt;
  }
  return model.measuredPositions(
      payload.at("name").get<std::vector<std::string>>(),
      payload.at("position").get<std::vector<double>>());
}

void sendJoints(zenoh::Session &session, const std::string &key,
                const zms::manipulation::JointModel &model,
                const std::vector<double> &positions) {
  Json names = Json::array();
  for (const auto &joint : model.joints()) {
    names.push_back(joint.name);
  }
  zms::contract::publish(
      session, key,
      {{"contract", 1}, {"name", names}, {"position", positions}});
}

int run(const Config &config, const zms::manipulation::JointModel &model) {
  zenoh::Session session(zenoh::Config::create_default());
  const auto endpoint = [&](std::string_view suffix) {
    return zms::contract::key(config.robotPrefix, suffix);
  };
  const auto targetKey = endpoint("manipulation/target");
  const auto cancelKey = endpoint("manipulation/cancel");
  const auto statusKey = endpoint("manipulation/status");
  const auto feedbackKey = endpoint("manipulation/feedback");
  const auto resultKey = endpoint("manipulation/result");
  const auto commandKey = endpoint("cmd_joints");
  Mailbox mailbox;

  const auto callback = [&](Request::Kind kind) {
    return [&, kind](const std::optional<Json> &payload) -> Json {
      if (!payload) {
        throw std::invalid_argument("request payload is required");
      }
      if (kind == Request::Kind::Target) {
        zms::contract::parseTarget(*payload);
      } else {
        zms::contract::parseCancel(*payload);
      }
      auto request = std::make_shared<Request>();
      request->kind = kind;
      request->payload = *payload;
      auto future = request->reply.get_future();
      {
        std::lock_guard lock(mailbox.mutex);
        if (mailbox.requests.size() >= 32) {
          throw std::runtime_error("request queue is full");
        }
        mailbox.requests.push_back(request);
      }
      if (future.wait_for(std::chrono::seconds(2)) !=
          std::future_status::ready) {
        auto expected = Claim::Waiting;
        if (request->claim.compare_exchange_strong(expected,
                                                   Claim::Abandoned)) {
          throw std::runtime_error("control loop did not answer in time");
        }
      }
      return future.get();
    };
  };
  auto targetQueryable = zms::contract::declareJsonQueryable(
      session, targetKey, callback(Request::Kind::Target));
  auto cancelQueryable = zms::contract::declareJsonQueryable(
      session, cancelKey, callback(Request::Kind::Cancel));
  auto statusQueryable = zms::contract::declareJsonQueryable(
      session, statusKey, [&](const std::optional<Json> &) {
        std::lock_guard lock(mailbox.mutex);
        auto status = mailbox.status;
        status["stack_version"] = "0.1.0";
        status["features"] = Json::array({"urdf_joint_limits", "joint_target"});
        return status;
      });
  auto subscriber = session.declare_subscriber(
      zenoh::KeyExpr(endpoint("joint_states")),
      [&](const zenoh::Sample &sample) {
        try {
          const auto state = orderedState(
              zms::contract::parseJson(sample.get_payload()), model);
          if (state) {
            std::lock_guard lock(mailbox.mutex);
            mailbox.state = std::pair{*state, Clock::now()};
          }
        } catch (const std::exception &) {
        }
      },
      [] {});
  auto alive = session.liveliness_declare_token(
      zenoh::KeyExpr(endpoint("manipulation/alive")));

  std::optional<zms::contract::Target> active;
  std::vector<double> measured;
  std::vector<double> commanded;
  std::vector<double> velocity;
  std::vector<double> goal;
  std::vector<double> initial;
  Clock::time_point lastState;
  const auto period = std::chrono::duration<double>(1.0 / config.controlRateHz);
  auto next = Clock::now();
  while (!stopRequested) {
    const auto now = Clock::now();
    std::deque<std::shared_ptr<Request>> requests;
    bool stateUpdated = false;
    {
      std::lock_guard lock(mailbox.mutex);
      requests.swap(mailbox.requests);
      if (mailbox.state) {
        measured = std::move(mailbox.state->first);
        lastState = mailbox.state->second;
        mailbox.state.reset();
        stateUpdated = true;
      }
    }
    for (const auto &request : requests) {
      auto expected = Claim::Waiting;
      if (!request->claim.compare_exchange_strong(expected, Claim::Handled)) {
        continue;
      }
      if (request->kind == Request::Kind::Target) {
        const auto target = zms::contract::parseTarget(request->payload);
        if (!std::holds_alternative<zms::contract::JointTarget>(target.value)) {
          request->reply.set_value(zms::contract::rejected(
              target.commandId, target.clientId, "unsupported",
              "pose targets require the pending IK planner"));
          continue;
        }
        std::vector<double> positions;
        try {
          positions = model.orderedPositions(
              std::get<zms::contract::JointTarget>(target.value));
        } catch (const std::exception &error) {
          request->reply.set_value(
              zms::contract::rejected(target.commandId, target.clientId,
                                      "invalid_target", error.what()));
          continue;
        }
        if (active && active->clientId != target.clientId) {
          request->reply.set_value(
              zms::contract::rejected(target.commandId, target.clientId, "busy",
                                      "another client owns the command"));
          continue;
        }
        if (measured.empty() ||
            now - lastState >
                std::chrono::duration<double>(config.stateTimeoutSeconds)) {
          request->reply.set_value(zms::contract::rejected(
              target.commandId, target.clientId, "state_unavailable",
              "joint_states are unavailable or stale"));
          continue;
        }
        if (active) {
          sendJoints(session, commandKey, model, measured);
          zms::contract::publish(
              session, resultKey,
              zms::contract::status(active->commandId, active->clientId,
                                    zms::contract::CommandState::Cancelled,
                                    "replaced", "replaced by owner"));
        }
        active = target;
        goal = std::move(positions);
        commanded = measured;
        initial = measured;
        velocity.assign(goal.size(), 0.0);
        request->reply.set_value(
            zms::contract::accepted(target.commandId, target.clientId));
      } else {
        const auto cancel = zms::contract::parseCancel(request->payload);
        if (!active) {
          request->reply.set_value({{"contract", 1}, {"state", "idle"}});
        } else if (active->clientId != cancel.clientId ||
                   active->commandId != cancel.commandId) {
          request->reply.set_value(zms::contract::rejected(
              cancel.commandId, cancel.clientId, "not_owner",
              "command belongs to another client"));
        } else {
          sendJoints(session, commandKey, model, measured);
          zms::contract::publish(
              session, resultKey,
              zms::contract::status(active->commandId, active->clientId,
                                    zms::contract::CommandState::Cancelled));
          active.reset();
          request->reply.set_value(
              zms::contract::accepted(cancel.commandId, cancel.clientId));
        }
      }
    }
    if (active) {
      std::string failure;
      if (measured.empty() ||
          now - lastState >
              std::chrono::duration<double>(config.stateTimeoutSeconds)) {
        failure = "stale_joint_states";
      } else if (stateUpdated && !commanded.empty()) {
        for (std::size_t index = 0; index < goal.size(); ++index) {
          if (std::abs(measured[index] - commanded[index]) >
              config.trackingError) {
            failure = "tracking_error";
          }
        }
      }
      if (!failure.empty()) {
        if (!measured.empty()) {
          sendJoints(session, commandKey, model, measured);
        }
        zms::contract::publish(
            session, resultKey,
            zms::contract::status(active->commandId, active->clientId,
                                  zms::contract::CommandState::Failed, failure,
                                  "joint execution stopped"));
        active.reset();
      } else {
        bool reached = true;
        double tracking = 0.0;
        for (std::size_t index = 0; index < goal.size(); ++index) {
          const auto step = zms::manipulation::stepTowards(
              commanded[index], velocity[index], goal[index],
              model.joints()[index].velocity, config.acceleration,
              1.0 / config.controlRateHz);
          commanded[index] = step.position;
          velocity[index] = step.velocity;
          commanded[index] =
              std::clamp(commanded[index], model.joints()[index].lower,
                         model.joints()[index].upper);
          reached &= std::abs(goal[index] - measured[index]) <=
                     config.positionTolerance;
          tracking =
              std::max(tracking, std::abs(measured[index] - commanded[index]));
        }
        sendJoints(session, commandKey, model, commanded);
        if (reached) {
          zms::contract::publish(
              session, resultKey,
              zms::contract::status(active->commandId, active->clientId,
                                    zms::contract::CommandState::Succeeded));
          active.reset();
        } else {
          Json feedback =
              zms::contract::status(active->commandId, active->clientId,
                                    zms::contract::CommandState::Active);
          feedback["tracking_error"] = tracking;
          double total = 0.0;
          double remaining = 0.0;
          for (std::size_t index = 0; index < goal.size(); ++index) {
            total += std::abs(goal[index] - initial[index]);
            remaining += std::abs(goal[index] - measured[index]);
          }
          feedback["progress"] =
              total > 0.0 ? std::clamp(1.0 - remaining / total, 0.0, 1.0) : 1.0;
          zms::contract::publish(session, feedbackKey, feedback);
        }
      }
    }
    {
      std::lock_guard lock(mailbox.mutex);
      mailbox.status =
          active ? zms::contract::status(active->commandId, active->clientId,
                                         zms::contract::CommandState::Active)
                 : Json{{"contract", 1}, {"state", "idle"}};
    }
    next += std::chrono::duration_cast<Clock::duration>(period);
    std::this_thread::sleep_until(next);
    if (next < Clock::now()) {
      next = Clock::now();
    }
  }
  if (!measured.empty()) {
    sendJoints(session, commandKey, model, measured);
  }
  if (active) {
    zms::contract::publish(
        session, resultKey,
        zms::contract::status(active->commandId, active->clientId,
                              zms::contract::CommandState::Failed, "shutdown",
                              "stack stopped"));
  }
  return 0;
}

}

int main(int argc, char **argv) {
  try {
    if (argc == 2 && std::string_view(argv[1]) == "--version") {
      std::cout << Json{{"stack", "manipulation"},
                        {"version", "0.1.0"},
                        {"contract", 1},
                        {"config_versions", Json::array({1})}}
                       .dump()
                << '\n';
      return 0;
    }
    if ((argc == 3 && std::string_view(argv[1]) == "--check-config") ||
        ((argc == 3 || argc == 5) && std::string_view(argv[1]) == "--config")) {
      const auto config = loadConfig(argv[2]);
      const auto model = zms::manipulation::JointModel::fromUrdf(
          config.urdfPath, config.baseFrame, config.endEffectorFrame);
      if (std::string_view(argv[1]) == "--check-config") {
        std::cout
            << Json{{"valid", true}, {"joints", model.joints().size()}}.dump()
            << '\n';
        return 0;
      }
      if (argc == 5 && (std::string_view(argv[3]) != "--robot-prefix" ||
                        std::string_view(argv[4]) != config.robotPrefix)) {
        throw std::invalid_argument(
            "robot prefix disagrees with configuration");
      }
      std::signal(SIGINT, requestStop);
      std::signal(SIGTERM, requestStop);
      return run(config, model);
    }
    throw std::invalid_argument("usage: zms_manipulation --config <file> "
                                "[--robot-prefix <prefix>] | "
                                "--check-config <file> | --version");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
