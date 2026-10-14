#include <zms/contract/Contract.hpp>
#include <zms/contract/ZenohClient.hpp>
#include <zms/manipulation/Collision.hpp>
#include <zms/manipulation/JointModel.hpp>
#include <zms/manipulation/Kinematics.hpp>
#include <zms/manipulation/Planner.hpp>

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
  double planningSeconds = 2.0;
  double collisionPadding = 0.01;
  double ikPositionTolerance = 0.003;
  double ikOrientationTolerance = 0.02;
  double settleTimeoutSeconds = 2.0;
  std::vector<zms::manipulation::EnvironmentObject> environment;
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

std::vector<double> numberArray(const Json &object, const char *field,
                                std::size_t count, const std::string &where) {
  if (!object.contains(field) || !object.at(field).is_array() ||
      object.at(field).size() != count) {
    throw std::invalid_argument(where + "." + field + " must hold " +
                                std::to_string(count) + " numbers");
  }
  std::vector<double> values;
  for (const auto &item : object.at(field)) {
    if (!item.is_number() || !std::isfinite(item.get<double>())) {
      throw std::invalid_argument(where + "." + field +
                                  " must hold finite numbers");
    }
    values.push_back(item.get<double>());
  }
  return values;
}

void knownFields(const Json &object,
                 const std::unordered_set<std::string> &allowed,
                 const std::string &where) {
  if (!object.is_object()) {
    throw std::invalid_argument(where + " must be an object");
  }
  for (auto item = object.begin(); item != object.end(); ++item) {
    if (!allowed.contains(item.key())) {
      throw std::invalid_argument(where + "." + item.key() + " is unknown");
    }
  }
}

zms::manipulation::EnvironmentObject parseObstacle(const Json &item,
                                                   const std::string &where) {
  knownFields(item, {"name", "type", "size", "position", "orientation"}, where);
  zms::manipulation::EnvironmentObject object;
  if (!item.contains("name") || !item.at("name").is_string() ||
      item.at("name").get<std::string>().empty()) {
    throw std::invalid_argument(where + ".name must be a nonempty string");
  }
  object.name = item.at("name").get<std::string>();
  const std::string type = item.contains("type") && item.at("type").is_string()
                               ? item.at("type").get<std::string>()
                               : std::string();
  std::vector<double> size;
  if (type == "box") {
    object.kind = zms::manipulation::ShapeKind::Box;
    size = numberArray(item, "size", 3, where);
    object.size = Eigen::Vector3d(size[0], size[1], size[2]);
  } else if (type == "cylinder") {
    object.kind = zms::manipulation::ShapeKind::Cylinder;
    size = numberArray(item, "size", 2, where);
    object.size = Eigen::Vector3d(size[0], size[1], 0.0);
  } else if (type == "sphere") {
    object.kind = zms::manipulation::ShapeKind::Sphere;
    size = numberArray(item, "size", 1, where);
    object.size = Eigen::Vector3d(size[0], 0.0, 0.0);
  } else {
    throw std::invalid_argument(where +
                                ".type must be box, cylinder, or sphere");
  }
  if (std::any_of(size.begin(), size.end(),
                  [](double value) { return value <= 0.0; })) {
    throw std::invalid_argument(where + ".size must be positive");
  }
  const auto position = numberArray(item, "position", 3, where);
  object.pose.translation() =
      Eigen::Vector3d(position[0], position[1], position[2]);
  if (item.contains("orientation")) {
    const auto q = numberArray(item, "orientation", 4, where);
    const Eigen::Quaterniond rotation(q[3], q[0], q[1], q[2]);
    if (std::abs(rotation.norm() - 1.0) > 0.001) {
      throw std::invalid_argument(where +
                                  ".orientation must be a unit quaternion");
    }
    object.pose.linear() = rotation.normalized().toRotationMatrix();
  }
  return object;
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
      "version",          "robot_prefix",   "urdf_path",
      "base_frame",       "eef_frame",      "control_rate_hz",
      "state_timeout_s",  "tracking_error", "position_tolerance",
      "max_acceleration", "planning"};
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
  if (document.contains("planning")) {
    const auto &planning = document.at("planning");
    knownFields(planning,
                {"max_time_s", "collision_padding", "ik_position_tolerance",
                 "ik_orientation_tolerance", "settle_timeout_s", "environment"},
                "config.planning");
    result.planningSeconds =
        positive(planning, "max_time_s", result.planningSeconds);
    if (planning.contains("collision_padding")) {
      if (!planning.at("collision_padding").is_number() ||
          planning.at("collision_padding").get<double>() < 0.0) {
        throw std::invalid_argument(
            "config.planning.collision_padding must be a nonnegative number");
      }
      result.collisionPadding = planning.at("collision_padding").get<double>();
    }
    result.ikPositionTolerance =
        positive(planning, "ik_position_tolerance", result.ikPositionTolerance);
    result.ikOrientationTolerance = positive(
        planning, "ik_orientation_tolerance", result.ikOrientationTolerance);
    result.settleTimeoutSeconds =
        positive(planning, "settle_timeout_s", result.settleTimeoutSeconds);
    if (planning.contains("environment")) {
      if (!planning.at("environment").is_array() ||
          planning.at("environment").size() > 256) {
        throw std::invalid_argument(
            "config.planning.environment must be an array of up to 256 "
            "objects");
      }
      for (std::size_t index = 0; index < planning.at("environment").size();
           ++index) {
        result.environment.push_back(parseObstacle(
            planning.at("environment").at(index),
            "config.planning.environment[" + std::to_string(index) + "]"));
      }
    }
  }
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

struct Planning {
  zms::manipulation::KinematicChain chain;
  zms::manipulation::CollisionModel collision;
};

Planning makePlanning(const Config &config,
                      const zms::manipulation::JointModel &model) {
  auto chain = zms::manipulation::KinematicChain::fromUrdf(
      config.urdfPath, config.baseFrame, config.endEffectorFrame);
  std::vector<double> reference;
  for (const auto &joint : model.joints()) {
    reference.push_back(std::clamp(0.0, joint.lower, joint.upper));
  }
  auto collision = zms::manipulation::CollisionModel::fromUrdf(
      config.urdfPath, config.baseFrame, chain, reference,
      config.collisionPadding);
  collision.setEnvironment(config.environment);
  return {std::move(chain), std::move(collision)};
}

Eigen::Isometry3d toIsometry(const zms::contract::PoseTarget &pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);
  result.linear() = Eigen::Quaterniond(pose.qw, pose.qx, pose.qy, pose.qz)
                        .normalized()
                        .toRotationMatrix();
  return result;
}

int run(const Config &config, const zms::manipulation::JointModel &model,
        const Planning &planning) {
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
  const auto valid = [&planning](const std::vector<double> &positions) {
    return !planning.collision.inCollision(positions);
  };
  zms::manipulation::PlannerOptions plannerOptions;
  plannerOptions.maxSeconds = config.planningSeconds;
  zms::manipulation::PlanningThread planner(valid, model.joints(),
                                            plannerOptions);

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
        status["stack_version"] = "0.2.0";
        status["features"] =
            Json::array({"urdf_joint_limits", "joint_target", "pose_target",
                         "self_collision", "environment_collision",
                         "rrt_connect", "synchronized_trajectory"});
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
  std::optional<std::uint64_t> planGeneration;
  std::optional<zms::manipulation::Trajectory> trajectory;
  Clock::time_point trajectoryStarted;
  std::vector<double> measured;
  std::vector<double> commanded;
  std::vector<double> goal;
  Clock::time_point lastState;
  const auto period = std::chrono::duration<double>(1.0 / config.controlRateHz);
  const auto stale = [&](Clock::time_point now) {
    return measured.empty() ||
           now - lastState >
               std::chrono::duration<double>(config.stateTimeoutSeconds);
  };
  const auto finish = [&](zms::contract::CommandState state,
                          std::string_view reason, std::string_view message) {
    if (!measured.empty() && (trajectory || planGeneration)) {
      sendJoints(session, commandKey, model, measured);
    }
    zms::contract::publish(session, resultKey,
                           zms::contract::status(active->commandId,
                                                 active->clientId, state,
                                                 reason, message));
    planner.cancel();
    planGeneration.reset();
    trajectory.reset();
    active.reset();
  };
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
      if (request->kind == Request::Kind::Cancel) {
        const auto cancel = zms::contract::parseCancel(request->payload);
        if (!active) {
          request->reply.set_value({{"contract", 1}, {"state", "idle"}});
        } else if (active->clientId != cancel.clientId ||
                   active->commandId != cancel.commandId) {
          request->reply.set_value(zms::contract::rejected(
              cancel.commandId, cancel.clientId, "not_owner",
              "command belongs to another client"));
        } else {
          finish(zms::contract::CommandState::Cancelled, {}, {});
          request->reply.set_value(
              zms::contract::accepted(cancel.commandId, cancel.clientId));
        }
        continue;
      }
      const auto target = zms::contract::parseTarget(request->payload);
      const auto reject = [&](std::string_view reason, std::string message) {
        request->reply.set_value(zms::contract::rejected(
            target.commandId, target.clientId, reason, message));
      };
      if (active && active->clientId != target.clientId) {
        reject("busy", "another client owns the command");
        continue;
      }
      if (stale(now)) {
        reject("state_unavailable", "joint_states are unavailable or stale");
        continue;
      }
      std::vector<double> desired;
      if (const auto *joints =
              std::get_if<zms::contract::JointTarget>(&target.value)) {
        try {
          desired = model.orderedPositions(*joints);
        } catch (const std::exception &error) {
          reject("invalid_target", error.what());
          continue;
        }
        std::string detail;
        if (planning.collision.inCollision(desired, &detail)) {
          reject("invalid_target", "target is in collision: " + detail);
          continue;
        }
      } else {
        const auto &pose = std::get<zms::contract::PoseTarget>(target.value);
        if ((!pose.baseFrame.empty() && pose.baseFrame != config.baseFrame) ||
            (!pose.endEffectorFrame.empty() &&
             pose.endEffectorFrame != config.endEffectorFrame)) {
          reject("unsupported", "frame overrides must match the configured "
                                "base_frame and eef_frame");
          continue;
        }
        zms::manipulation::IkOptions options;
        options.positionTolerance = config.ikPositionTolerance;
        options.orientationTolerance = config.ikOrientationTolerance;
        const auto solution = planning.chain.solveIk(
            toIsometry(pose), measured, model.joints(), options, valid);
        if (!solution) {
          reject("unreachable",
                 "no collision-free joint solution reaches the pose");
          continue;
        }
        desired = *solution;
      }
      if (active) {
        finish(zms::contract::CommandState::Cancelled, "replaced",
               "replaced by owner");
      }
      active = target;
      goal = desired;
      commanded = measured;
      planGeneration = planner.submit(measured, desired);
      Json reply = zms::contract::accepted(target.commandId, target.clientId);
      reply["joints"] = {{"name", Json::array()}, {"position", desired}};
      for (const auto &joint : model.joints()) {
        reply["joints"]["name"].push_back(joint.name);
      }
      request->reply.set_value(reply);
    }
    if (active && stale(now)) {
      finish(zms::contract::CommandState::Failed, "stale_joint_states",
             "joint execution stopped");
    }
    if (active && planGeneration) {
      if (auto outcome = planner.poll();
          outcome && outcome->generation == *planGeneration) {
        planGeneration.reset();
        if (!outcome->path) {
          finish(zms::contract::CommandState::Failed,
                 outcome->error == "start_in_collision" ? "start_in_collision"
                                                        : "planning_failed",
                 outcome->error);
        } else {
          trajectory = zms::manipulation::timeParameterize(
              *outcome->path, model.joints(), config.acceleration);
          trajectoryStarted = now;
        }
      }
    }
    if (active && trajectory) {
      const double elapsed =
          std::chrono::duration<double>(now - trajectoryStarted).count();
      commanded = trajectory->sample(elapsed);
      double tracking = 0.0;
      bool reached = true;
      for (std::size_t index = 0; index < goal.size(); ++index) {
        tracking =
            std::max(tracking, std::abs(measured[index] - commanded[index]));
        reached &=
            std::abs(goal[index] - measured[index]) <= config.positionTolerance;
      }
      if (stateUpdated && tracking > config.trackingError) {
        finish(zms::contract::CommandState::Failed, "tracking_error",
               "joint execution stopped");
      } else {
        sendJoints(session, commandKey, model, commanded);
        if (elapsed >= trajectory->duration() && reached) {
          finish(zms::contract::CommandState::Succeeded, {}, {});
        } else if (elapsed >=
                   trajectory->duration() + config.settleTimeoutSeconds) {
          finish(zms::contract::CommandState::Failed, "goal_not_reached",
                 "joints did not settle within settle_timeout_s");
        } else {
          Json feedback =
              zms::contract::status(active->commandId, active->clientId,
                                    zms::contract::CommandState::Active);
          feedback["tracking_error"] = tracking;
          feedback["progress"] =
              trajectory->duration() > 0.0
                  ? std::clamp(elapsed / trajectory->duration(), 0.0, 1.0)
                  : 1.0;
          zms::contract::publish(session, feedbackKey, feedback);
        }
      }
    }
    {
      std::lock_guard lock(mailbox.mutex);
      mailbox.status =
          active ? zms::contract::status(
                       active->commandId, active->clientId,
                       trajectory ? zms::contract::CommandState::Active
                                  : zms::contract::CommandState::Pending)
                 : Json{{"contract", 1}, {"state", "idle"}};
    }
    next += std::chrono::duration_cast<Clock::duration>(period);
    std::this_thread::sleep_until(next);
    if (next < Clock::now()) {
      next = Clock::now();
    }
  }
  if (active) {
    finish(zms::contract::CommandState::Failed, "shutdown", "stack stopped");
  } else if (!measured.empty()) {
    sendJoints(session, commandKey, model, measured);
  }
  return 0;
}

}

int main(int argc, char **argv) {
  try {
    if (argc == 2 && std::string_view(argv[1]) == "--version") {
      std::cout << Json{{"stack", "manipulation"},
                        {"version", "0.2.0"},
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
      const auto planning = makePlanning(config, model);
      if (std::string_view(argv[1]) == "--check-config") {
        std::cout << Json{{"valid", true},
                          {"joints", model.joints().size()},
                          {"collision_geometries",
                           planning.collision.robotGeometryCount()},
                          {"self_collision_pairs",
                           planning.collision.selfPairCount()},
                          {"skipped_meshes",
                           planning.collision.skippedMeshCount()},
                          {"environment_objects", config.environment.size()}}
                         .dump()
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
      return run(config, model, planning);
    }
    throw std::invalid_argument("usage: zms_manipulation --config <file> "
                                "[--robot-prefix <prefix>] | "
                                "--check-config <file> | --version");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
