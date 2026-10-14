#include <zms/navigation/Navigation.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace zms::navigation {
namespace {

double limitChange(double current, double desired, double maximumChange) {
  return current + std::clamp(desired - current, -maximumChange, maximumChange);
}

}

double normalizeAngle(double angle) {
  return std::remainder(angle, 2.0 * std::numbers::pi);
}

double nearestForwardObstacle(const std::vector<double> &ranges,
                              double angleMin, double angleIncrement,
                              double rangeMin, double rangeMax) {
  if (ranges.empty() || ranges.size() > 8192 || !std::isfinite(angleMin) ||
      !std::isfinite(angleIncrement) || angleIncrement <= 0.0 ||
      !std::isfinite(rangeMin) || rangeMin < 0.0 || !std::isfinite(rangeMax) ||
      rangeMax <= rangeMin) {
    throw std::invalid_argument("invalid scan geometry or bounds");
  }
  double nearest = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < ranges.size(); ++index) {
    const double angle = normalizeAngle(angleMin + index * angleIncrement);
    const double range = ranges[index];
    if (std::abs(angle) <= std::numbers::pi / 2.0 && std::isfinite(range) &&
        range >= rangeMin && range <= rangeMax) {
      nearest = std::min(nearest, range);
    }
  }
  return nearest;
}

StepResult follow(const Pose &pose,
                  const std::vector<contract::Waypoint> &waypoints,
                  std::size_t waypointIndex, const Velocity &previous,
                  const Limits &limits, double elapsedSeconds,
                  bool enforceIntermediateYaw) {
  if (waypoints.empty() || waypointIndex >= waypoints.size()) {
    throw std::invalid_argument("active route has no current waypoint");
  }
  if (!std::isfinite(elapsedSeconds) || elapsedSeconds <= 0.0) {
    throw std::invalid_argument("elapsedSeconds must be positive");
  }
  const auto &target = waypoints[waypointIndex];
  const double dx = target.x - pose.x;
  const double dy = target.y - pose.y;
  const double distance = std::hypot(dx, dy);
  const double yawError = normalizeAngle(target.yaw - pose.yaw);
  const bool needsYaw =
      enforceIntermediateYaw || waypointIndex + 1 == waypoints.size();
  if (distance <= limits.positionTolerance &&
      (!needsYaw || std::abs(yawError) <= limits.yawTolerance)) {
    const std::size_t next = waypointIndex + 1;
    return {{}, next == waypoints.size(), next};
  }
  double desiredLinear = 0.0;
  double desiredAngular = 0.0;
  if (distance <= limits.positionTolerance) {
    desiredAngular = std::clamp(yawError * 2.0, -limits.maxAngularSpeed,
                                limits.maxAngularSpeed);
  } else {
    const double headingError = normalizeAngle(std::atan2(dy, dx) - pose.yaw);
    if (std::abs(headingError) > std::numbers::pi / 3.0) {
      desiredAngular = std::clamp(headingError * 2.0, -limits.maxAngularSpeed,
                                  limits.maxAngularSpeed);
    } else {
      desiredLinear = std::min(
          {limits.maxLinearSpeed, distance * 0.8,
           std::sqrt(2.0 * limits.maxLinearAcceleration *
                     std::max(0.0, distance - limits.positionTolerance))});
      const double curvature = 2.0 * std::sin(headingError) /
                               std::max(distance, limits.lookaheadDistance);
      desiredAngular =
          std::clamp(desiredLinear * curvature, -limits.maxAngularSpeed,
                     limits.maxAngularSpeed);
    }
  }
  return {{limitChange(previous.linear, desiredLinear,
                       limits.maxLinearAcceleration * elapsedSeconds),
           limitChange(previous.angular, desiredAngular,
                       limits.maxAngularAcceleration * elapsedSeconds)},
          false,
          waypointIndex};
}

}
