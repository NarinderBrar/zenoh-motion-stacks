#include <zms/contract/Contract.hpp>
#include <zms/contract/ZenohClient.hpp>
#include <zms/navigation/LocalPlanner.hpp>
#include <zms/navigation/Localization.hpp>
#include <zms/navigation/Navigation.hpp>
#include <zms/navigation/PlanningWorker.hpp>
#include <zms/navigation/Visualization.hpp>

#include <zenoh.hxx>

#include <algorithm>
#include <atomic>
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

struct VisualizationConfig {
  bool enabled = true;
  double costmapRateHz = 1.0;
  double pathRateHz = 5.0;
};

struct Config {
  std::string robotPrefix;
  int controlRateHz = 20;
  double odomTimeoutSeconds = 1.0;
  double scanTimeoutSeconds = 1.0;
  double scanIgnoreRadius = 0.0;
  double footprintRadius = 0.25;
  double obstacleMargin = 0.1;
  double progressTimeoutSeconds = 15.0;
  bool avoidance = true;
  double avoidanceHorizonSeconds = 1.5;
  double recoveryAfterSeconds = 2.0;
  double localWindowMeters = 6.0;
  zms::navigation::Pose scanOffset;
  bool localization = false;
  int localizationParticles = 400;
  std::optional<zms::navigation::Pose> initialPose;
  zms::navigation::Limits limits;
  VisualizationConfig visualization;
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

double nonnegativeNumber(const Json &object, const char *field,
                         double fallback) {
  if (!object.contains(field)) {
    return fallback;
  }
  if (!object.at(field).is_number()) {
    throw std::invalid_argument(std::string(field) + " must be a number");
  }
  const double value = object.at(field).get<double>();
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(field) +
                                " must be nonnegative and finite");
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
               "scan_timeout_s", "scan_ignore_radius", "footprint_radius",
               "obstacle_margin", "progress_timeout_s", "avoidance",
               "scan_offset", "localization", "limits", "visualization"},
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
  result.scanIgnoreRadius =
      nonnegativeNumber(document, "scan_ignore_radius", result.scanIgnoreRadius);
  result.footprintRadius =
      positiveNumber(document, "footprint_radius", result.footprintRadius);
  result.obstacleMargin =
      positiveNumber(document, "obstacle_margin", result.obstacleMargin);
  result.progressTimeoutSeconds = positiveNumber(document, "progress_timeout_s",
                                                 result.progressTimeoutSeconds);
  if (document.contains("avoidance")) {
    const auto &avoidance = document.at("avoidance");
    knownFields(avoidance,
                {"enabled", "horizon_s", "recovery_after_s", "local_window_m"},
                "config.avoidance");
    if (avoidance.contains("enabled")) {
      if (!avoidance.at("enabled").is_boolean()) {
        throw std::invalid_argument(
            "config.avoidance.enabled must be a boolean");
      }
      result.avoidance = avoidance.at("enabled").get<bool>();
    }
    result.avoidanceHorizonSeconds =
        positiveNumber(avoidance, "horizon_s", result.avoidanceHorizonSeconds);
    result.recoveryAfterSeconds = positiveNumber(avoidance, "recovery_after_s",
                                                 result.recoveryAfterSeconds);
    result.localWindowMeters =
        positiveNumber(avoidance, "local_window_m", result.localWindowMeters);
  }
  const auto poseField = [](const Json &object, const std::string &where) {
    knownFields(object, {"x", "y", "yaw"}, where);
    zms::navigation::Pose pose;
    for (const auto &[name, target] :
         {std::pair{"x", &pose.x}, std::pair{"y", &pose.y},
          std::pair{"yaw", &pose.yaw}}) {
      if (object.contains(name)) {
        if (!object.at(name).is_number() ||
            !std::isfinite(object.at(name).get<double>())) {
          throw std::invalid_argument(where + "." + name +
                                      " must be a finite number");
        }
        *target = object.at(name).get<double>();
      }
    }
    return pose;
  };
  if (document.contains("scan_offset")) {
    result.scanOffset =
        poseField(document.at("scan_offset"), "config.scan_offset");
  }
  if (document.contains("localization")) {
    const auto &localization = document.at("localization");
    knownFields(localization, {"enabled", "particles", "initial_pose"},
                "config.localization");
    if (localization.contains("enabled")) {
      if (!localization.at("enabled").is_boolean()) {
        throw std::invalid_argument(
            "config.localization.enabled must be a boolean");
      }
      result.localization = localization.at("enabled").get<bool>();
    }
    if (localization.contains("particles")) {
      if (!localization.at("particles").is_number_integer() ||
          localization.at("particles").get<int>() < 50 ||
          localization.at("particles").get<int>() > 5000) {
        throw std::invalid_argument(
            "config.localization.particles must be an integer from 50 to 5000");
      }
      result.localizationParticles = localization.at("particles").get<int>();
    }
    if (localization.contains("initial_pose")) {
      result.initialPose = poseField(localization.at("initial_pose"),
                                     "config.localization.initial_pose");
    }
  }
  if (document.contains("visualization")) {
    const auto &viz = document.at("visualization");
    knownFields(viz, {"enabled", "costmap_rate_hz", "path_rate_hz"},
                "config.visualization");
    if (viz.contains("enabled")) {
      if (!viz.at("enabled").is_boolean()) {
        throw std::invalid_argument(
            "config.visualization.enabled must be a boolean");
      }
      result.visualization.enabled = viz.at("enabled").get<bool>();
    }
    result.visualization.costmapRateHz =
        positiveNumber(viz, "costmap_rate_hz", result.visualization.costmapRateHz);
    result.visualization.pathRateHz =
        positiveNumber(viz, "path_rate_hz", result.visualization.pathRateHz);
  }
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

enum class Claim { Waiting, Handled, Abandoned };

struct Request {
  enum class Kind { Goal, Cancel } kind;
  Json payload;
  std::promise<Json> reply;
  std::atomic<Claim> claim = Claim::Waiting;
};

struct Mailbox {
  std::mutex mutex;
  std::deque<std::shared_ptr<Request>> requests;
  std::deque<std::pair<zms::navigation::Pose, Clock::time_point>> odometry;
  struct Scan {
    double nearest = 0.0;
    std::vector<zms::navigation::Point2> points;
    Clock::time_point received;
  };
  std::deque<Scan> scans;
  std::deque<std::shared_ptr<const zms::navigation::OccupancyGrid>> maps;
  std::optional<zms::navigation::Pose> initialPose;
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
  zenoh::Session session(zms::contract::sessionConfig());
  const auto endpoint = [&](std::string_view suffix) {
    return zms::contract::key(config.robotPrefix, suffix);
  };
  const std::string goalKey = endpoint("navigation/goal");
  const std::string cancelKey = endpoint("navigation/cancel");
  const std::string statusKey = endpoint("navigation/status");
  const std::string feedbackKey = endpoint("navigation/feedback");
  const std::string resultKey = endpoint("navigation/result");
  const std::string velocityKey = endpoint("cmd_vel");
  const std::string pathKey = endpoint("navigation/path");
  const std::string localPathKey = endpoint("navigation/local_path");
  const std::string costmapKey = endpoint("navigation/costmap");
  const std::string obstaclesKey = endpoint("navigation/obstacles");
  Mailbox mailbox;
  zms::navigation::PlanningWorker planner;
  const auto publishEmptyPaths = [&](const std::string &commandId) {
    if (!config.visualization.enabled) {
      return;
    }
    const Json empty = {{"contract", 1},
                        {"command_id", commandId},
                        {"frame", "map"},
                        {"poses", Json::array()}};
    zms::contract::publish(session, pathKey, empty);
    zms::contract::publish(session, localPathKey, empty);
  };

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
        current["stack_version"] = "0.2.0";
        current["features"] =
            config.avoidance
                ? Json::array({"waypoint_route", "differential_drive",
                               "grid_astar", "local_avoidance",
                               "obstacle_replanning", "recovery_behaviors"})
                : Json::array({"waypoint_route", "differential_drive",
                               "scan_obstacle_stop", "grid_astar"});
        if (config.localization) {
          current["features"].push_back("particle_filter_localization");
        }
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
          auto points = zms::navigation::outsideRadius(
              zms::navigation::toWorld(
                  zms::navigation::scanToPoints(
                      ranges, value.at("angle_min").get<double>(),
                      value.at("angle_increment").get<double>(),
                      value.at("range_min").get<double>(),
                      value.at("range_max").get<double>()),
                  config.scanOffset),
              config.scanIgnoreRadius);
          double nearest = std::numeric_limits<double>::infinity();
          for (const auto &point : points) {
            if (point.x >= 0.0) {
              nearest = std::min(nearest, std::hypot(point.x, point.y));
            }
          }
          std::lock_guard lock(mailbox.mutex);
          if (mailbox.scans.size() == 4) {
            mailbox.scans.pop_front();
          }
          mailbox.scans.push_back({nearest, std::move(points), Clock::now()});
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
  auto initialPoseSubscriber = session.declare_subscriber(
      zenoh::KeyExpr(endpoint("localization/initial_pose")),
      [&](const zenoh::Sample &sample) {
        try {
          const Json value = zms::contract::parseJson(sample.get_payload());
          if (!value.contains("contract") || value.at("contract") != 1) {
            return;
          }
          const zms::navigation::Pose initial{value.at("x").get<double>(),
                                              value.at("y").get<double>(),
                                              value.at("yaw").get<double>()};
          if (!std::isfinite(initial.x) || !std::isfinite(initial.y) ||
              !std::isfinite(initial.yaw)) {
            return;
          }
          std::lock_guard lock(mailbox.mutex);
          mailbox.initialPose = initial;
        } catch (const std::exception &) {
        }
      },
      [] {});
  const std::string localizationPoseKey = endpoint("localization/pose");
  zms::navigation::LocalizationConfig localizationConfig;
  localizationConfig.particles = config.localizationParticles;
  zms::navigation::ParticleFilter filter(localizationConfig);
  std::shared_ptr<const zms::navigation::OccupancyGrid> filterMap;
  std::optional<zms::navigation::Pose> odomPose;
  bool odomUpdated = false;
  Clock::time_point lastLocalizationPublish;
  auto aliveToken = session.liveliness_declare_token(
      zenoh::KeyExpr(endpoint("navigation/alive")));
  std::optional<zms::contract::Goal> active;
  std::optional<zms::navigation::Pose> pose;
  std::optional<double> nearestScan;
  std::vector<zms::navigation::Point2> scanPoints;
  zms::navigation::ObstacleMemory obstacleMemory;
  enum class Recovery { None, Rotate, Backup };
  Recovery recovery = Recovery::None;
  bool blocked = false;
  Clock::time_point blockedSince;
  Clock::time_point recoveryStarted;
  double recoveryDirection = 1.0;
  zms::navigation::Pose recoveryStartPose;
  std::size_t waypointOffset = 0;
  bool replanning = false;
  std::size_t replanOffset = 0;
  zms::navigation::LocalPlannerConfig localConfig;
  localConfig.robotRadius = config.footprintRadius;
  localConfig.safetyMargin = config.obstacleMargin;
  localConfig.horizonSeconds = config.avoidanceHorizonSeconds;
  const auto resetAvoidance = [&] {
    recovery = Recovery::None;
    blocked = false;
  };
  std::shared_ptr<const zms::navigation::OccupancyGrid> latestMap;
  std::vector<zms::contract::Waypoint> route;
  std::vector<std::size_t> routeWaypointIndices;
  bool routePlannedFromMap = false;
  std::optional<std::uint64_t> planGeneration;
  Clock::time_point lastOdom;
  Clock::time_point lastScan;
  std::size_t waypointIndex = 0;
  bool aligning = false;
  zms::navigation::ProgressMonitor progress(config.progressTimeoutSeconds);
  const auto loopStarted = Clock::now();
  zms::navigation::Velocity previous;
  Clock::time_point commandStarted = Clock::now();
  Clock::time_point lastPathPublish = Clock::time_point();
  Clock::time_point lastLocalPathPublish = Clock::time_point();
  Clock::time_point lastCostmapPublish = Clock::time_point();
  Clock::time_point lastObstaclesPublish = Clock::time_point();
  zms::navigation::OccupancyGrid lastPublishedCostmap;
  bool hasPublishedCostmap = false;
  std::vector<zms::contract::Waypoint> lastPublishedRoute;
  const auto submittedWaypointIndex = [&] {
    if (routePlannedFromMap && waypointIndex < routeWaypointIndices.size()) {
      return waypointOffset + routeWaypointIndices[waypointIndex];
    }
    return waypointIndex;
  };
  const auto period = std::chrono::duration<double>(1.0 / config.controlRateHz);
  auto next = Clock::now();
  auto previousTick = next;

  while (!stopRequested) {
    const auto now = Clock::now();
    const double nowSeconds =
        std::chrono::duration<double>(now.time_since_epoch()).count();
    bool scanUpdated = false;
    std::deque<std::shared_ptr<Request>> requests;
    {
      std::lock_guard lock(mailbox.mutex);
      requests.swap(mailbox.requests);
      if (mailbox.initialPose && config.localization) {
        filter.initialize(*mailbox.initialPose);
      }
      mailbox.initialPose.reset();
      if (!mailbox.odometry.empty()) {
        odomPose = mailbox.odometry.back().first;
        odomUpdated = true;
        lastOdom = mailbox.odometry.back().second;
        mailbox.odometry.clear();
      }
      if (!mailbox.scans.empty()) {
        scanUpdated = true;
        nearestScan = mailbox.scans.back().nearest;
        scanPoints = std::move(mailbox.scans.back().points);
        lastScan = mailbox.scans.back().received;
        mailbox.scans.clear();
      }
      if (!mailbox.maps.empty()) {
        latestMap = mailbox.maps.back();
        mailbox.maps.clear();
      }
    }
    if (config.localization) {
      if (latestMap && latestMap != filterMap) {
        filterMap = latestMap;
        filter.setMap(filterMap);
      }
      if (!filter.initialized() && config.initialPose) {
        filter.initialize(*config.initialPose);
      }
      if (odomUpdated && odomPose) {
        filter.update(*odomPose, scanPoints);
      }
      if (odomPose && filter.initialized() &&
          now - lastLocalizationPublish > std::chrono::milliseconds(200)) {
        lastLocalizationPublish = now;
        const auto estimate = filter.toMap(*odomPose);
        zms::contract::publish(session, localizationPoseKey,
                               {{"contract", 1},
                                {"x", estimate.x},
                                {"y", estimate.y},
                                {"yaw", estimate.yaw},
                                {"spread", filter.positionSpread()}});
      }
    }
    if (odomPose) {
      pose = config.localization && filter.initialized()
                 ? filter.toMap(*odomPose)
                 : *odomPose;
    }
    if (scanUpdated && pose) {
      obstacleMemory.update(zms::navigation::toWorld(scanPoints, *pose),
                            nowSeconds);
    }
    odomUpdated = false;
    for (const auto &request : requests) {
      auto expected = Claim::Waiting;
      if (!request->claim.compare_exchange_strong(expected, Claim::Handled)) {
        continue;
      }
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
          publishEmptyPaths(active->commandId);
        }
        planner.cancel();
        active = incoming;
        route = latestMap ? std::vector<zms::contract::Waypoint>{}
                          : incoming.waypoints;
        routePlannedFromMap = false;
        planGeneration.reset();
        commandStarted = now;
        waypointIndex = 0;
        waypointOffset = 0;
        replanning = false;
        aligning = false;
        progress.reset();
        resetAvoidance();
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
          publishEmptyPaths(active->commandId);
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
      if (!outcome->error.empty() && replanning) {
        replanning = false;
      } else if (!outcome->error.empty()) {
        publishVelocity(session, velocityKey, {});
        publishResult(session, resultKey, *active,
                      zms::contract::CommandState::Failed, "planning_failed",
                      outcome->error);
        publishEmptyPaths(active->commandId);
        active.reset();
        route.clear();
      } else {
        route = std::move(outcome->route);
        routeWaypointIndices = std::move(outcome->waypointIndices);
        routePlannedFromMap = true;
        waypointIndex = 0;
        waypointOffset = replanning ? replanOffset : 0;
        replanning = false;
        aligning = false;
        progress.reset();
        resetAvoidance();
      }
    }
    if (active) {
      if (nearestScan && std::chrono::duration<double>(now - lastScan).count() >
                             config.scanTimeoutSeconds) {
        publishVelocity(session, velocityKey, {});
        publishResult(session, resultKey, *active,
                      zms::contract::CommandState::Failed, "stale_scan",
                      "scan stopped updating");
        publishEmptyPaths(active->commandId);
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
        publishEmptyPaths(active->commandId);
        active.reset();
        route.clear();
        planGeneration.reset();
        planner.cancel();
        previous = {};
      } else if (!route.empty() && waypointIndex < route.size() &&
                 progress.stalled(
                     waypointIndex,
                     std::hypot(route[waypointIndex].x - pose->x,
                                route[waypointIndex].y - pose->y),
                     zms::navigation::normalizeAngle(route[waypointIndex].yaw -
                                                     pose->yaw),
                     std::chrono::duration<double>(now - loopStarted)
                         .count())) {
        publishVelocity(session, velocityKey, {});
        publishResult(session, resultKey, *active,
                      zms::contract::CommandState::Failed, "no_progress",
                      "no progress toward the current waypoint within "
                      "progress_timeout_s");
        publishEmptyPaths(active->commandId);
        active.reset();
        route.clear();
        planGeneration.reset();
        planner.cancel();
        previous = {};
      } else if (!config.avoidance && nearestScan &&
                 *nearestScan <
                     config.footprintRadius + config.obstacleMargin) {
        publishVelocity(session, velocityKey, {});
        previous = {};
        Json feedback =
            zms::contract::status(active->commandId, active->clientId,
                                  zms::contract::CommandState::Active);
        feedback["waypoint_index"] = submittedWaypointIndex();
        feedback["obstacle_stopped"] = true;
        feedback["x"] = pose->x;
        feedback["y"] = pose->y;
        feedback["yaw"] = pose->yaw;
        zms::contract::publish(session, feedbackKey, feedback);
      } else if (route.empty()) {
        publishVelocity(session, velocityKey, {});
        previous = {};
      } else {
        const double elapsed = std::max(
            0.001, std::chrono::duration<double>(now - previousTick).count());
        zms::navigation::StepResult step;
        const bool avoiding = config.avoidance && nearestScan.has_value();
        if (avoiding && routePlannedFromMap) {
          waypointIndex = zms::navigation::lookaheadIndex(
              *pose, route, waypointIndex, config.limits.positionTolerance);
        }
        const auto &current = route[waypointIndex];
        const double toCurrent =
            std::hypot(current.x - pose->x, current.y - pose->y);
        const auto obstacles = avoiding
                                   ? obstacleMemory.points(nowSeconds)
                                   : std::vector<zms::navigation::Point2>{};
        const bool directClear =
            avoiding && recovery == Recovery::None &&
            zms::navigation::segmentClear(
                *pose,
                route[routePlannedFromMap ? zms::navigation::lookaheadIndex(
                                                *pose, route, waypointIndex,
                                                config.limits.lookaheadDistance)
                                          : waypointIndex],
                obstacles, config.footprintRadius + config.obstacleMargin);
        if (!avoiding || aligning || directClear ||
            toCurrent <= config.limits.positionTolerance) {
          step = zms::navigation::follow(*pose, route, waypointIndex, previous,
                                         config.limits, elapsed,
                                         !routePlannedFromMap, aligning);
          aligning = step.waypointIndex == waypointIndex && step.aligning;
          resetAvoidance();
        } else {
          step.waypointIndex = waypointIndex;
          if (recovery == Recovery::Rotate) {
            step.velocity = {0.0, recoveryDirection * 0.5 *
                                      config.limits.maxAngularSpeed};
            if (std::chrono::duration<double>(now - recoveryStarted).count() >
                2.0) {
              const double rearClearance =
                  config.footprintRadius + config.obstacleMargin + 0.25;
              const bool rearClear = std::none_of(
                  scanPoints.begin(), scanPoints.end(), [&](const auto &point) {
                    return point.x < 0.0 &&
                           std::hypot(point.x, point.y) < rearClearance;
                  });
              recovery = rearClear ? Recovery::Backup : Recovery::None;
              recoveryStarted = now;
              recoveryStartPose = *pose;
              blocked = false;
            }
          } else if (recovery == Recovery::Backup) {
            step.velocity = {-0.1, 0.0};
            if (std::hypot(pose->x - recoveryStartPose.x,
                           pose->y - recoveryStartPose.y) > 0.2 ||
                std::chrono::duration<double>(now - recoveryStarted).count() >
                    3.0) {
              recovery = Recovery::None;
              blocked = false;
            }
          } else {
            const std::size_t targetIndex =
                routePlannedFromMap ? zms::navigation::lookaheadIndex(
                                          *pose, route, waypointIndex,
                                          config.limits.lookaheadDistance)
                                    : waypointIndex;
            auto local = zms::navigation::planLocalPath(
                *pose, route[targetIndex], obstacles,
                config.footprintRadius + config.obstacleMargin,
                config.localWindowMeters);
            const auto &stopPoint =
                routePlannedFromMap ? route.back() : route[waypointIndex];
            const auto plan = local ? zms::navigation::chooseVelocity(
                                          *pose, previous, obstacles, *local,
                                          std::hypot(stopPoint.x - pose->x,
                                                     stopPoint.y - pose->y),
                                          config.limits, localConfig, elapsed)
                                    : zms::navigation::LocalPlan{};
            const bool moving =
                plan.found && (plan.target.linear > 0.01 ||
                               std::abs(plan.target.angular) > 0.05);
            step.velocity =
                moving ? plan.velocity : zms::navigation::Velocity{};
            if (moving) {
              blocked = false;
            } else if (!blocked) {
              blocked = true;
              blockedSince = now;
            } else if (std::chrono::duration<double>(now - blockedSince)
                           .count() > config.recoveryAfterSeconds) {
              if (latestMap && !planGeneration) {
                auto grid = std::make_shared<zms::navigation::OccupancyGrid>(
                    *latestMap);
                for (const auto &point : obstacles) {
                  const double dx = point.x - grid->origin.x;
                  const double dy = point.y - grid->origin.y;
                  const double cosine = std::cos(grid->origin.yaw);
                  const double sine = std::sin(grid->origin.yaw);
                  const int x = static_cast<int>(
                      std::floor((cosine * dx + sine * dy) / grid->resolution));
                  const int y = static_cast<int>(std::floor(
                      (-sine * dx + cosine * dy) / grid->resolution));
                  if (x >= 0 && y >= 0 && x < grid->width && y < grid->height) {
                    grid->cells[static_cast<std::size_t>(y) * grid->width + x] =
                        100;
                  }
                }
                replanOffset = submittedWaypointIndex();
                std::vector<zms::contract::Waypoint> remaining(
                    active->waypoints.begin() +
                        static_cast<std::ptrdiff_t>(replanOffset),
                    active->waypoints.end());
                planGeneration = planner.submit(
                    {grid, *pose, remaining,
                     config.footprintRadius + config.obstacleMargin});
                replanning = true;
              }
              recovery = Recovery::Rotate;
              recoveryStarted = now;
              recoveryDirection =
                  zms::navigation::freeTurnDirection(*pose, obstacles);
            }
          }
        }
        waypointIndex = step.waypointIndex;
        previous = step.velocity;
        publishVelocity(session, velocityKey, step.velocity);
        if (step.reached) {
          publishResult(session, resultKey, *active,
                        zms::contract::CommandState::Succeeded);
          publishEmptyPaths(active->commandId);
          active.reset();
          route.clear();
          planGeneration.reset();
        } else {
          Json feedback =
              zms::contract::status(active->commandId, active->clientId,
                                    zms::contract::CommandState::Active);
          feedback["waypoint_index"] = submittedWaypointIndex();
          feedback["x"] = pose->x;
          feedback["y"] = pose->y;
          feedback["yaw"] = pose->yaw;
          feedback["blocked"] = blocked;
          feedback["recovery"] = recovery == Recovery::Rotate   ? "rotate"
                                 : recovery == Recovery::Backup ? "backup"
                                                                : "none";
          zms::contract::publish(session, feedbackKey, feedback);
        }
      }
    }
    if (config.visualization.enabled) {
      const auto pathPeriod = std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(1.0 /
                                        std::max(0.1, config.visualization.pathRateHz)));
      if (active && (route != lastPublishedRoute ||
                     now - lastPathPublish > pathPeriod)) {
        lastPathPublish = now;
        lastPublishedRoute = route;
        zms::contract::publish(session, pathKey,
                               {{"contract", 1},
                                {"command_id", active->commandId},
                                {"frame", "map"},
                                {"poses", zms::navigation::posesToJson(route)}});
      }
      if (active && now - lastLocalPathPublish > pathPeriod) {
        lastLocalPathPublish = now;
        std::vector<zms::contract::Waypoint> localRoute;
        if (pose && (blocked || recovery != Recovery::None) && !route.empty()) {
          if (routePlannedFromMap) {
            const std::size_t startIndex = zms::navigation::lookaheadIndex(
                *pose, route, waypointIndex, 0.0);
            for (std::size_t i = startIndex;
                 i < route.size() && i < startIndex + 64; ++i) {
              localRoute.push_back(route[i]);
            }
          } else if (waypointIndex < route.size()) {
            localRoute.push_back(route[waypointIndex]);
          }
        }
        zms::contract::publish(session, localPathKey,
                               {{"contract", 1},
                                {"command_id", active->commandId},
                                {"frame", "map"},
                                {"poses", zms::navigation::posesToJson(localRoute)}});
      }
      const auto costmapPeriod = std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(
              1.0 / std::max(0.1, config.visualization.costmapRateHz)));
      if (now - lastCostmapPublish > costmapPeriod) {
        lastCostmapPublish = now;
        bool haveGrid = false;
        zms::navigation::OccupancyGrid grid;
        if (latestMap) {
          grid = zms::navigation::inflateGrid(*latestMap,
                             config.footprintRadius + config.obstacleMargin);
          zms::navigation::markPoints(grid, obstacleMemory.points(nowSeconds));
          haveGrid = true;
        } else if (pose) {
          const double resolution =
              std::clamp(config.localWindowMeters / 200.0, 0.02, 0.1);
          grid = zms::navigation::buildLocalWindowGrid(*pose, config.localWindowMeters,
                                      resolution,
                                      config.footprintRadius +
                                          config.obstacleMargin,
                                      obstacleMemory.points(nowSeconds));
          haveGrid = true;
        }
        if (haveGrid) {
          constexpr int maximumCostmapCells = 40000;
          int stride = 1;
          const auto bounded = zms::navigation::downsampleGrid(grid, maximumCostmapCells, stride);
          if (!hasPublishedCostmap || !zms::navigation::sameGrid(bounded, lastPublishedCostmap)) {
            lastPublishedCostmap = bounded;
            hasPublishedCostmap = true;
            Json payload;
            payload = zms::navigation::gridToJson(bounded);
            zms::contract::publish(session, costmapKey, payload);
          }
        }
      }
      const auto obstaclesPeriod = std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(1.0 / 5.0));
      if (now - lastObstaclesPublish > obstaclesPeriod) {
        lastObstaclesPublish = now;
        Json payload = {{"contract", 1}, {"frame", "map"}, {"points", Json::array()}};
        const auto memory = obstacleMemory.points(nowSeconds);
        constexpr std::size_t maximumPoints = 10000;
        for (std::size_t i = 0; i < memory.size() && i < maximumPoints; ++i) {
          payload["points"].push_back({{"x", memory[i].x}, {"y", memory[i].y}});
        }
        zms::contract::publish(session, obstaclesKey, payload);
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
      mailbox.status["localized"] =
          config.localization && filter.initialized() && filter.hasMap();
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
    publishEmptyPaths(active->commandId);
  }
  return 0;
}

}

int main(int argc, char **argv) {
  try {
    if (argc == 2 && std::string_view(argv[1]) == "--version") {
      std::cout << Json{{"stack", "navigation"},
                        {"version", "0.2.0"},
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
