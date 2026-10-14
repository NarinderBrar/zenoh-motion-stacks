#include <zms/navigation/GridPlanner.hpp>
#include <zms/navigation/LocalPlanner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace zms::navigation {
namespace {

Pose integrate(const Pose &pose, double linear, double angular, double dt) {
  if (std::abs(angular) < 1e-9) {
    return {pose.x + linear * std::cos(pose.yaw) * dt,
            pose.y + linear * std::sin(pose.yaw) * dt, pose.yaw};
  }
  const double yaw = pose.yaw + angular * dt;
  const double radius = linear / angular;
  return {pose.x + radius * (std::sin(yaw) - std::sin(pose.yaw)),
          pose.y - radius * (std::cos(yaw) - std::cos(pose.yaw)),
          normalizeAngle(yaw)};
}

}

std::vector<Point2> scanToPoints(const std::vector<double> &ranges,
                                 double angleMin, double angleIncrement,
                                 double rangeMin, double rangeMax) {
  if (!std::isfinite(angleMin) || !std::isfinite(angleIncrement) ||
      angleIncrement <= 0.0 || !std::isfinite(rangeMin) || rangeMin < 0.0 ||
      !std::isfinite(rangeMax) || rangeMax <= rangeMin ||
      ranges.size() > 8192) {
    throw std::invalid_argument("invalid scan geometry or bounds");
  }
  std::vector<Point2> points;
  points.reserve(ranges.size());
  for (std::size_t index = 0; index < ranges.size(); ++index) {
    const double range = ranges[index];
    if (!std::isfinite(range) || range < rangeMin || range > rangeMax) {
      continue;
    }
    const double angle = angleMin + static_cast<double>(index) * angleIncrement;
    points.push_back({range * std::cos(angle), range * std::sin(angle)});
  }
  return points;
}

std::vector<Point2> toWorld(const std::vector<Point2> &robotPoints,
                            const Pose &pose) {
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  std::vector<Point2> world;
  world.reserve(robotPoints.size());
  for (const auto &point : robotPoints) {
    world.push_back({pose.x + cosine * point.x - sine * point.y,
                     pose.y + sine * point.x + cosine * point.y});
  }
  return world;
}

std::vector<Point2> outsideRadius(const std::vector<Point2> &points,
                                  double radius) {
  std::vector<Point2> filtered;
  filtered.reserve(points.size());
  for (const auto &point : points) {
    if (std::hypot(point.x, point.y) >= radius) {
      filtered.push_back(point);
    }
  }
  return filtered;
}

void ObstacleMemory::update(const std::vector<Point2> &points,
                            double nowSeconds) {
  this->points(nowSeconds);
  for (const auto &point : points) {
    auto match =
        std::find_if(entries_.begin(), entries_.end(), [&](const Entry &entry) {
          return std::hypot(entry.point.x - point.x, entry.point.y - point.y) <
                 0.08;
        });
    if (match != entries_.end()) {
      match->seenSeconds = nowSeconds;
    } else {
      entries_.push_back({point, nowSeconds});
    }
  }
  if (entries_.size() > 1024) {
    std::nth_element(entries_.begin(), entries_.begin() + 1024, entries_.end(),
                     [](const Entry &a, const Entry &b) {
                       return a.seenSeconds > b.seenSeconds;
                     });
    entries_.resize(1024);
  }
}

std::vector<Point2> ObstacleMemory::points(double nowSeconds) {
  std::erase_if(entries_, [&](const Entry &entry) {
    return nowSeconds - entry.seenSeconds > 3.0;
  });
  std::vector<Point2> result;
  result.reserve(entries_.size());
  for (const auto &entry : entries_) {
    result.push_back(entry.point);
  }
  return result;
}

double clearanceAt(const Pose &pose, const std::vector<Point2> &obstacles) {
  double nearest = std::numeric_limits<double>::infinity();
  for (const auto &point : obstacles) {
    nearest = std::min(nearest, std::hypot(point.x - pose.x, point.y - pose.y));
  }
  return nearest;
}

bool segmentClear(const Pose &from, const contract::Waypoint &to,
                  const std::vector<Point2> &obstacles, double clearance) {
  const double length = std::hypot(to.x - from.x, to.y - from.y);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / 0.05)));
  for (int step = 0; step <= steps; ++step) {
    const double ratio = static_cast<double>(step) / steps;
    const Pose sample{from.x + (to.x - from.x) * ratio,
                      from.y + (to.y - from.y) * ratio, from.yaw};
    if (clearanceAt(sample, obstacles) < clearance) {
      return false;
    }
  }
  return true;
}

LocalPlan chooseVelocity(const Pose &pose, const Velocity &current,
                         const std::vector<Point2> &obstacles,
                         const std::vector<contract::Waypoint> &guidePath,
                         double distanceToGoal, const Limits &limits,
                         const LocalPlannerConfig &config,
                         double controlPeriod) {
  if (controlPeriod <= 0.0 || config.linearSamples < 2 ||
      config.angularSamples < 2 || config.simulationStep <= 0.0 ||
      config.horizonSeconds < config.simulationStep) {
    throw std::invalid_argument("invalid local planner configuration");
  }
  std::vector<Point2> nearby;
  const double reach = config.sensingRange + config.robotRadius;
  for (const auto &point : obstacles) {
    if (std::hypot(point.x - pose.x, point.y - pose.y) <= reach) {
      nearby.push_back(point);
    }
  }
  const double minimumClearance = config.robotRadius + config.safetyMargin;
  const double window = std::max(controlPeriod, config.windowSeconds);
  const double linearLow =
      std::max(0.0, current.linear - limits.maxLinearAcceleration * window);
  const double linearHigh =
      std::min({limits.maxLinearSpeed,
                current.linear + limits.maxLinearAcceleration * window,
                std::sqrt(2.0 * limits.maxLinearAcceleration *
                          std::max(0.0, distanceToGoal))});
  const double angularLow =
      std::max(-limits.maxAngularSpeed,
               current.angular - limits.maxAngularAcceleration * window);
  const double angularHigh =
      std::min(limits.maxAngularSpeed,
               current.angular + limits.maxAngularAcceleration * window);
  const int steps = static_cast<int>(
      std::round(config.horizonSeconds / config.simulationStep));
  if (guidePath.empty()) {
    throw std::invalid_argument("local planner needs a target path");
  }
  std::vector<contract::Waypoint> path = {guidePath.front()};
  for (std::size_t index = 1; index < guidePath.size(); ++index) {
    const auto &from = guidePath[index - 1];
    const auto &to = guidePath[index];
    const double length = std::hypot(to.x - from.x, to.y - from.y);
    const int pieces = std::max(1, static_cast<int>(std::ceil(length / 0.05)));
    for (int piece = 1; piece <= pieces; ++piece) {
      const double ratio = static_cast<double>(piece) / pieces;
      path.push_back({from.x + (to.x - from.x) * ratio,
                      from.y + (to.y - from.y) * ratio, to.yaw});
    }
  }
  const contract::Waypoint &target = path.back();
  const double startDistance = std::hypot(target.x - pose.x, target.y - pose.y);
  std::vector<double> along(path.size(), 0.0);
  for (std::size_t index = 1; index < path.size(); ++index) {
    along[index] =
        along[index - 1] + std::hypot(path[index].x - path[index - 1].x,
                                      path[index].y - path[index - 1].y);
  }
  const auto closestFrom = [&](const Pose &query, std::size_t first) {
    std::size_t best = first;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (std::size_t index = first; index < path.size(); ++index) {
      const double distance =
          std::hypot(path[index].x - query.x, path[index].y - query.y);
      if (distance < bestDistance) {
        bestDistance = distance;
        best = index;
      }
    }
    return best;
  };
  const std::size_t startIndex = closestFrom(pose, 0);
  const double reachable =
      std::max(1e-6, limits.maxLinearSpeed * config.horizonSeconds);

  LocalPlan best;
  double bestScore = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < config.linearSamples; ++i) {
    const double linear = linearHigh <= linearLow
                              ? linearLow
                              : linearLow + (linearHigh - linearLow) * i /
                                                (config.linearSamples - 1);
    for (int j = 0; j < config.angularSamples; ++j) {
      const double angular = angularLow + (angularHigh - angularLow) * j /
                                              (config.angularSamples - 1);
      Pose simulated = pose;
      double previous = clearanceAt(simulated, nearby);
      double clearance = previous;
      bool collides = false;
      for (int step = 0; step < steps && !collides; ++step) {
        simulated =
            integrate(simulated, linear, angular, config.simulationStep);
        const double here = clearanceAt(simulated, nearby);
        clearance = std::min(clearance, here);
        collides = here < minimumClearance && here < previous - 1e-6;
        previous = here;
      }
      if (collides) {
        continue;
      }
      const double stopping =
          linear * linear / (2.0 * limits.maxLinearAcceleration);
      if (std::isfinite(clearance) && clearance >= minimumClearance &&
          stopping > clearance - minimumClearance + 1e-9) {
        continue;
      }
      double score = config.clearanceWeight * std::min(clearance, 1.0) +
                     config.speedWeight * linear / limits.maxLinearSpeed;
      if (path.size() >= 2) {
        const std::size_t endIndex = closestFrom(simulated, startIndex);
        std::size_t aheadIndex = endIndex;
        while (aheadIndex + 1 < path.size() &&
               along[aheadIndex] < along[endIndex] + 0.3) {
          ++aheadIndex;
        }
        const double offPath = std::hypot(path[endIndex].x - simulated.x,
                                          path[endIndex].y - simulated.y);
        const double heading = std::abs(
            normalizeAngle(std::atan2(path[aheadIndex].y - simulated.y,
                                      path[aheadIndex].x - simulated.x) -
                           simulated.yaw));
        score += config.progressWeight * (along[endIndex] - along[startIndex]) /
                     reachable -
                 config.pathWeight * offPath / 0.5 -
                 config.headingWeight * heading / std::numbers::pi;
      } else {
        const double endDistance =
            std::hypot(target.x - simulated.x, target.y - simulated.y);
        const double progress =
            startDistance > 1e-9 ? (startDistance - endDistance) / startDistance
                                 : 0.0;
        const double heading = std::abs(normalizeAngle(
            std::atan2(target.y - simulated.y, target.x - simulated.x) -
            simulated.yaw));
        score += config.progressWeight * progress -
                 config.headingWeight * heading / std::numbers::pi;
      }
      if (score > bestScore) {
        bestScore = score;
        best = {true, {linear, angular}, {}, clearance};
      }
    }
  }
  if (best.found) {
    const double linearStep = limits.maxLinearAcceleration * controlPeriod;
    const double angularStep = limits.maxAngularAcceleration * controlPeriod;
    best.velocity = {
        current.linear + std::clamp(best.target.linear - current.linear,
                                    -linearStep, linearStep),
        current.angular + std::clamp(best.target.angular - current.angular,
                                     -angularStep, angularStep)};
  }
  return best;
}

std::size_t lookaheadIndex(const Pose &pose,
                           const std::vector<contract::Waypoint> &route,
                           std::size_t current, double lookahead) {
  if (route.empty()) {
    return 0;
  }
  std::size_t index = std::min(current, route.size() - 1);
  double closest = std::numeric_limits<double>::infinity();
  for (std::size_t candidate = index; candidate < route.size(); ++candidate) {
    const double distance =
        std::hypot(route[candidate].x - pose.x, route[candidate].y - pose.y);
    if (distance < closest) {
      closest = distance;
      index = candidate;
    }
  }
  while (index + 1 < route.size() &&
         std::hypot(route[index].x - pose.x, route[index].y - pose.y) <
             lookahead) {
    ++index;
  }
  return index;
}

std::optional<std::vector<contract::Waypoint>>
planLocalPath(const Pose &pose, const contract::Waypoint &target,
              const std::vector<Point2> &obstacles, double clearance,
              double windowSize, double resolution) {
  if (windowSize <= 0.0 || resolution <= 0.0 || clearance < 0.0) {
    throw std::invalid_argument("invalid local path window");
  }
  const int cells = static_cast<int>(std::ceil(windowSize / resolution));
  OccupancyGrid grid;
  grid.width = cells;
  grid.height = cells;
  grid.resolution = resolution;
  const double half = cells * resolution / 2.0;
  grid.origin = {pose.x - half, pose.y - half, 0.0};
  grid.cells.assign(static_cast<std::size_t>(cells) * cells, 0);
  for (const auto &point : obstacles) {
    const int x =
        static_cast<int>(std::floor((point.x - grid.origin.x) / resolution));
    const int y =
        static_cast<int>(std::floor((point.y - grid.origin.y) / resolution));
    if (x >= 0 && y >= 0 && x < cells && y < cells) {
      grid.cells[static_cast<std::size_t>(y) * cells + x] = 100;
    }
  }
  contract::Waypoint goal = target;
  const double dx = target.x - pose.x;
  const double dy = target.y - pose.y;
  const double limit = half - 2.0 * resolution;
  const double largest = std::max(std::abs(dx), std::abs(dy));
  if (largest > limit) {
    goal.x = pose.x + dx * limit / largest;
    goal.y = pose.y + dy * limit / largest;
  }
  for (const double extra : {0.10, 0.06, 0.02, 0.0}) {
    try {
      return planGrid(grid, pose, goal, clearance + extra);
    } catch (const std::exception &) {
    }
  }
  return std::nullopt;
}

double freeTurnDirection(const Pose &pose,
                         const std::vector<Point2> &obstacles) {
  double left = 0.0;
  double right = 0.0;
  for (const auto &point : obstacles) {
    const double bearing = normalizeAngle(
        std::atan2(point.y - pose.y, point.x - pose.x) - pose.yaw);
    const double weight =
        1.0 / std::max(0.05, std::hypot(point.x - pose.x, point.y - pose.y));
    (bearing >= 0.0 ? left : right) += weight;
  }
  return left <= right ? 1.0 : -1.0;
}

}
