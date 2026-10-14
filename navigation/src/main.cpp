#include <zms/contract/Contract.hpp>
#include <zms/contract/ZenohClient.hpp>
#include <zms/navigation/Navigation.hpp>
#include <zms/navigation/PlanningWorker.hpp>

#include <zenoh.hxx>

#include <chrono>
#include <cmath>
#include <csignal>
#include <deque>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
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
  int controlRateHz = 20;
  double odomTimeoutSeconds = 1.0;
  double scanTimeoutSeconds = 1.0;
  double footprintRadius = 0.25;
  double obstacleMargin = 0.1;
  zms::navigation::Limits limits;
};

double positiveNumber(const Json &object, const char *field, double fallback) {
  if (!object.contains(field)) {
    return fallback;
  }
  if (!object.at(field).is_number()) {
    throw std::invalid_argument(std::string(field) + " must be a number");
  }
  const double value = object.at(field).get<double>();
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(field) +
                                " must be positive and finite");
  }
  return value;
}

void knownFields(const Json &object,
                 const std::unordered_set<std::string> &allowed,
                 const std::string &location) {
  if (!object.is_object()) {
    throw std::invalid_argument(location + " must be an object");
  }
  for (auto item = object.begin(); item != object.end(); ++item) {
    if (!allowed.contains(item.key())) {
      throw std::invalid_argument(location + "." + item.key() + " is unknown");
    }
  }
}

Config loadConfig(const std::string &path) {
  std::ifstream input(path);
  if (!input) {
    throw std::invalid_argument("cannot read configuration: " + path);
  }
  const Json document = Json::parse(input);
  knownFields(document,
              {"version", "robot_prefix", "control_rate_hz", "odom_timeout_s",
               "scan_timeout_s", "footprint_radius", "obstacle_margin",
               "limits"},
              "config");
  if (!document.contains("version") || document.at("version") != 1) {
    throw std::invalid_argument("config.version must be 1");
  }
  if (!document.contains("robot_prefix") ||
      !document.at("robot_prefix").is_string()) {
    throw std::invalid_argument("config.robot_prefix must be a string");
  }
  Config result;
  result.robotPrefix = document.at("robot_prefix").get<std::string>();
  zms::contract::key(result.robotPrefix, "odom");
  if (document.contains("control_rate_hz")) {
    if (!document.at("control_rate_hz").is_number_integer()) {
      throw std::invalid_argument("config.control_rate_hz must be an integer");
    }
    result.controlRateHz = document.at("control_rate_hz").get<int>();
    if (result.controlRateHz < 1 || result.controlRateHz > 200) {
      throw std::invalid_argument(
          "config.control_rate_hz must be from 1 to 200");
    }
  }
  result.odomTimeoutSeconds = positiveNumber(document, "odom_timeout_s", 1.0);
  result.scanTimeoutSeconds =
      positiveNumber(document, "scan_timeout_s", result.scanTimeoutSeconds);
  result.footprintRadius =
      positiveNumber(document, "footprint_radius", result.footprintRadius);
  result.obstacleMargin =
      positiveNumber(document, "obstacle_margin", result.obstacleMargin);
  if (document.contains("limits")) {
    const auto &limits = document.at("limits");
    knownFields(limits,
                {"max_linear_speed", "max_angular_speed",
                 "max_linear_acceleration", "max_angular_acceleration",
                 "position_tolerance", "yaw_tolerance", "lookahead_distance"},
                "config.limits");
    result.limits.maxLinearSpeed = positiveNumber(limits, "max_linear_speed",
                                                  result.limits.maxLinearSpeed);
    result.limits.maxAngularSpeed = positiveNumber(
        limits, "max_angular_speed", result.limits.maxAngularSpeed);
    result.limits.maxLinearAcceleration = positiveNumber(
        limits, "max_linear_acceleration", result.limits.maxLinearAcceleration);
    result.limits.maxAngularAcceleration =
        positiveNumber(limits, "max_angular_acceleration",
                       result.limits.maxAngularAcceleration);
    result.limits.positionTolerance = positiveNumber(
        limits, "position_tolerance", result.limits.positionTolerance);
    result.limits.yawTolerance =
        positiveNumber(limits, "yaw_tolerance", result.limits.yawTolerance);
    result.limits.lookaheadDistance = positiveNumber(
        limits, "lookahead_distance", result.limits.lookaheadDistance);
  }
  return result;
}

struct Request {
  enum class Kind { Goal, Cancel } kind;
  Json payload;
  std::promise<Json> reply;
};

struct Mailbox {
  std::mutex mutex;
  std::deque<std::shared_ptr<Request>> requests;
  std::deque<std::pair<zms::navigation::Pose, Clock::time_point>> odometry;
  std::deque<std::pair<double, Clock::time_point>> scans;
  std::deque<std::shared_ptr<const zms::navigation::OccupancyGrid>> maps;
  Json status = {{"contract", 1}, {"state", "idle"}};
};

void publishVelocity(zenoh::Session &session, const std::string &key,
                     const zms::navigation::Velocity &velocity) {
  zms::contract::publish(session, key,
                         {{"contract", 1},
                          {"linear", velocity.linear},
                          {"angular", velocity.angular}});
}

void publishResult(zenoh::Session &session, const std::string &key,
                   const zms::contract::Goal &goal,
                   zms::contract::CommandState state,
                   std::string_view reason = {},
                   std::string_view message = {}) {
  zms::contract::publish(session, key,
                         zms::contract::status(goal.commandId, goal.clientId,
                                               state, reason, message));
}

int run(const Config &config) {
  zenoh::Session session(zenoh::Config::create_default());
  const auto endpoint = [&](std::string_view suffix) {
    return zms::contract::key(config.robotPrefix, suffix);
  };
  const std::string goalKey = endpoint("navigation/goal");
  const std::string cancelKey = endpoint("navigation/cancel");
  const std::string statusKey = endpoint("navigation/status");
  const std::string feedbackKey = endpoint("navigation/feedback");
  const std::string resultKey = endpoint("navigation/result");
  const std::string velocityKey = endpoint("cmd_vel");
  Mailbox mailbox;
  zms::navigation::PlanningWorker planner;

  const auto commandCallback = [&](Request::Kind kind) {
    return [&, kind](const std::optional<Json> &payload) -> Json {
      if (!payload) {
        throw std::invalid_argument("request payload is required");
      }
      auto request = std::make_shared<Request>();
      request->kind = kind;
      request->payload = *payload;
      if (kind == Request::Kind::Goal) {
        zms::contract::parseGoal(request->payload);
      } else {
        zms::contract::parseCancel(request->payload);
      }
      auto future = request->reply.get_future();
      {
        std::lock_guard lock(mailbox.mutex);
        if (mailbox.requests.size() >= 32) {
          throw std::runtime_error("request queue is full");
        }
        mailbox.requests.push_back(std::move(request));
      }
      if (future.wait_for(std::chrono::seconds(2)) !=
          std::future_status::ready) {
        throw std::runtime_error("control loop did not answer in time");
      }
      return future.get();
    };
  };

  auto goalQueryable = zms::contract::declareJsonQueryable(
      session, goalKey, commandCallback(Request::Kind::Goal));
  auto cancelQueryable = zms::contract::declareJsonQueryable(
      session, cancelKey, commandCallback(Request::Kind::Cancel));
  auto statusQueryable = zms::contract::declareJsonQueryable(
      session, statusKey, [&](const std::optional<Json> &) {
        Json current;
        {
          std::lock_guard lock(mailbox.mutex);
          current = mailbox.status;
        }
        current["stack_version"] = "0.1.0";
        current["features"] =
            Json::array({"waypoint_route", "differential_drive",
                         "scan_obstacle_stop", "grid_astar"});
        return current;
      });
  auto odomSubscriber = session.declare_subscriber(
      zenoh::KeyExpr(endpoint("odom")),
      [&](const zenoh::Sample &sample) {
        try {
          const Json value = zms::contract::parseJson(sample.get_payload());
          const zms::navigation::Pose pose{value.at("x").get<double>(),
                                           value.at("y").get<double>(),
                                           value.at("yaw").get<double>()};
          if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
              !std::isfinite(pose.yaw)) {
            return;
          }
          std::lock_guard lock(mailbox.mutex);
          if (mailbox.odometry.size() == 32) {
            mailbox.odometry.pop_front();
          }
          mailbox.odometry.emplace_back(pose, Clock::now());
        } catch (const std::exception &) {
        }
      },
      [] {});
  auto scanSubscriber = session.declare_subscriber(
      zenoh::KeyExpr(endpoint("scan")),
      [&](const zenoh::Sample &sample) {
        try {
          const Json value =
              zms::contract::parseJson(sample.get_payload(), 1048576);
          if (!value.contains("contract") || value.at("contract") != 1 ||
              !value.contains("ranges") || !value.at("ranges").is_array() ||
              value.at("ranges").empty() || value.at("ranges").size() > 8192) {
            return;
          }
          std::vector<double> ranges;
          ranges.reserve(value.at("ranges").size());
          for (const auto &range : value.at("ranges")) {
            if (range.is_null()) {
              ranges.push_back(std::numeric_limits<double>::infinity());
            } else if (range.is_number()) {
              ranges.push_back(range.get<double>());
            } else {
              return;
            }
          }
          const double nearest = zms::navigation::nearestForwardObstacle(
              ranges, value.at("angle_min").get<double>(),
              value.at("angle_increment").get<double>(),
              value.at("range_min").get<double>(),
              value.at("range_max").get<double>());
          std::lock_guard lock(mailbox.mutex);
          if (mailbox.scans.size() == 32) {
            mailbox.scans.pop_front();
          }
          mailbox.scans.emplace_back(nearest, Clock::now());
        } catch (const std::exception &) {
        }
      },
      [] {});
  auto mapSubscriber = session.declare_subscriber(
      zenoh::KeyExpr(endpoint("map")),
      [&](const zenoh::Sample &sample) {
        try {
          const auto value =
              zms::contract::parseJson(sample.get_payload(), 8388608);
          auto map = std::make_shared<zms::navigation::OccupancyGrid>(
              zms::navigation::parseGrid(value));
          std::lock_guard lock(mailbox.mutex);
          if (mailbox.maps.size() == 2) {
            mailbox.maps.pop_front();
          }
          mailbox.maps.push_back(std::move(map));
        } catch (const std::exception &) {
        }
      },
      [] {});
  auto aliveToken = session.liveliness_declare_token(
      zenoh::KeyExpr(endpoint("navigation/alive")));
  std::optional<zms::contract::Goal> active;
  std::optional<zms::navigation::Pose> pose;
  std::optional<double> nearestScan;
  std::shared_ptr<const zms::navigation::OccupancyGrid> latestMap;
  std::vector<zms::contract::Waypoint> route;
  bool routePlannedFromMap = false;
  std::optional<std::uint64_t> planGeneration;
  Clock::time_point lastOdom;
  Clock::time_point lastScan;
  std::size_t waypointIndex = 0;
  zms::navigation::Velocity previous;
  Clock::time_point commandStarted = Clock::now();
  const auto period = std::chrono::duration<double>(1.0 / config.controlRateHz);
  auto next = Clock::now();
  auto previousTick = next;

  while (!stopRequested) {
    const auto now = Clock::now();
    std::deque<std::shared_ptr<Request>> requests;
    {
      std::lock_guard lock(mailbox.mutex);
      requests.swap(mailbox.requests);
      if (!mailbox.odometry.empty()) {
        pose = mailbox.odometry.back().first;
        lastOdom = mailbox.odometry.back().second;
        mailbox.odometry.clear();
      }
      if (!mailbox.scans.empty()) {
        nearestScan = mailbox.scans.back().first;
        lastScan = mailbox.scans.back().second;
        mailbox.scans.clear();
      }
      if (!mailbox.maps.empty()) {
        latestMap = mailbox.maps.back();
        mailbox.maps.clear();
      }
    }
    for (const auto &request : requests) {
      if (request->kind == Request::Kind::Goal) {
        const auto incoming = zms::contract::parseGoal(request->payload);
        if (active && active->clientId != incoming.clientId) {
          request->reply.set_value(zms::contract::rejected(
              incoming.commandId, incoming.clientId, "busy",
              "another client owns the command"));
          continue;
        }
        if (active) {
          publishVelocity(session, velocityKey, {});
          publishResult(session, resultKey, *active,
                        zms::contract::CommandState::Cancelled, "replaced",
                        "replaced by owner");
        }
        planner.cancel();
        active = incoming;
        route = latestMap ? std::vector<zms::contract::Waypoint>{}
                          : incoming.waypoints;
        routePlannedFromMap = false;
        planGeneration.reset();
        commandStarted = now;
        waypointIndex = 0;
        previous = {};
        request->reply.set_value(
            zms::contract::accepted(incoming.commandId, incoming.clientId));
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
          publishVelocity(session, velocityKey, {});
          publishResult(session, resultKey, *active,
                        zms::contract::CommandState::Cancelled);
          active.reset();
          route.clear();
          planGeneration.reset();
          planner.cancel();
          previous = {};
          request->reply.set_value(
              zms::contract::accepted(cancel.commandId, cancel.clientId));
        }
      }
    }
    if (active && latestMap && route.empty() && !planGeneration && pose) {
      planGeneration =
          planner.submit({latestMap, *pose, active->waypoints,
                          config.footprintRadius + config.obstacleMargin});
    }
    while (auto outcome = planner.poll()) {
      if (!active || !planGeneration ||
          outcome->generation != *planGeneration) {
        continue;
      }
      planGeneration.reset();
      if (!outcome->error.empty()) {
        publishVelocity(session, velocityKey, {});
        publishResult(session, resultKey, *active,
                      zms::contract::CommandState::Failed, "planning_failed",
                      outcome->error);
        active.reset();
        route.clear();
      } else {
        route = std::move(outcome->route);
        routePlannedFromMap = true;
        waypointIndex = 0;
      }
    }
    if (active) {
      if (nearestScan && std::chrono::duration<double>(now - lastScan).count() >
                             config.scanTimeoutSeconds) {
        publishVelocity(session, velocityKey, {});
        publishResult(session, resultKey, *active,
                      zms::contract::CommandState::Failed, "stale_scan",
                      "scan stopped updating");
        active.reset();
        route.clear();
        planGeneration.reset();
        planner.cancel();
        previous = {};
      } else if (!pose &&
                 std::chrono::duration<double>(now - commandStarted).count() <=
                     config.odomTimeoutSeconds) {
        publishVelocity(session, velocityKey, {});
      } else if (!pose ||
                 std::chrono::duration<double>(now - lastOdom).count() >
                     config.odomTimeoutSeconds) {
        publishVelocity(session, velocityKey, {});
        publishResult(session, resultKey, *active,
                      zms::contract::CommandState::Failed, "stale_odom",
                      "odometry is unavailable or stale");
        active.reset();
        route.clear();
        planGeneration.reset();
        planner.cancel();
        previous = {};
      } else if (nearestScan && *nearestScan < config.footprintRadius +
                                                   config.obstacleMargin) {
        publishVelocity(session, velocityKey, {});
        previous = {};
        Json feedback =
            zms::contract::status(active->commandId, active->clientId,
                                  zms::contract::CommandState::Active);
        feedback["waypoint_index"] = waypointIndex;
        feedback["obstacle_stopped"] = true;
        feedback["x"] = pose->x;
        feedback["y"] = pose->y;
        feedback["yaw"] = pose->yaw;
        zms::contract::publish(session, feedbackKey, feedback);
      } else if (route.empty()) {
        publishVelocity(session, velocityKey, {});
        previous = {};
      } else {
        const auto step = zms::navigation::follow(
            *pose, route, waypointIndex, previous, config.limits,
            std::max(0.001,
                     std::chrono::duration<double>(now - previousTick).count()),
            !routePlannedFromMap);
        waypointIndex = step.waypointIndex;
        previous = step.velocity;
        publishVelocity(session, velocityKey, step.velocity);
        if (step.reached) {
          publishResult(session, resultKey, *active,
                        zms::contract::CommandState::Succeeded);
          active.reset();
          route.clear();
          planGeneration.reset();
        } else {
          Json feedback =
              zms::contract::status(active->commandId, active->clientId,
                                    zms::contract::CommandState::Active);
          feedback["waypoint_index"] = waypointIndex;
          feedback["x"] = pose->x;
          feedback["y"] = pose->y;
          feedback["yaw"] = pose->yaw;
          zms::contract::publish(session, feedbackKey, feedback);
        }
      }
    }
    {
      std::lock_guard lock(mailbox.mutex);
      mailbox.status =
          active
              ? zms::contract::status(active->commandId, active->clientId,
                                      route.empty()
                                          ? zms::contract::CommandState::Pending
                                          : zms::contract::CommandState::Active)
              : Json{{"contract", 1}, {"state", "idle"}};
      mailbox.status["map_available"] = static_cast<bool>(latestMap);
    }
    previousTick = now;
    next += std::chrono::duration_cast<Clock::duration>(period);
    std::this_thread::sleep_until(next);
    if (next < Clock::now()) {
      next = Clock::now();
    }
  }
  publishVelocity(session, velocityKey, {});
  if (active) {
    publishResult(session, resultKey, *active,
                  zms::contract::CommandState::Failed, "shutdown",
                  "stack stopped");
  }
  return 0;
}

}

int main(int argc, char **argv) {
  try {
    if (argc == 2 && std::string_view(argv[1]) == "--version") {
      std::cout << Json{{"stack", "navigation"},
                        {"version", "0.1.0"},
                        {"contract", 1},
                        {"config_versions", Json::array({1})}}
                       .dump()
                << '\n';
      return 0;
    }
    if (argc == 3 && std::string_view(argv[1]) == "--check-config") {
      loadConfig(argv[2]);
      std::cout << Json{{"valid", true}}.dump() << '\n';
      return 0;
    }
    if (argc != 3 && argc != 5) {
      throw std::invalid_argument(
          "usage: zms_navigation --config <file> [--robot-prefix <prefix>]");
    }
    if (std::string_view(argv[1]) != "--config" ||
        (argc == 5 && std::string_view(argv[3]) != "--robot-prefix")) {
      throw std::invalid_argument("invalid arguments");
    }
    auto config = loadConfig(argv[2]);
    if (argc == 5) {
      const std::string overridePrefix = argv[4];
      if (overridePrefix != config.robotPrefix) {
        throw std::invalid_argument(
            "robot prefix disagrees with configuration");
      }
    }
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    return run(config);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
