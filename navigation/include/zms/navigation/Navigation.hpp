#pragma once

#include <zms/contract/Contract.hpp>

#include <cstddef>
#include <vector>

namespace zms::navigation {

struct Pose {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct Velocity {
  double linear = 0.0;
  double angular = 0.0;
};

struct Limits {
  double maxLinearSpeed = 0.5;
  double maxAngularSpeed = 1.0;
  double maxLinearAcceleration = 0.5;
  double maxAngularAcceleration = 1.0;
  double positionTolerance = 0.05;
  double yawTolerance = 0.1;
  double lookaheadDistance = 0.4;
};

struct StepResult {
  Velocity velocity;
  bool reached = false;
  std::size_t waypointIndex = 0;
  bool aligning = false;
};

class ProgressMonitor {
public:
  explicit ProgressMonitor(double timeoutSeconds);
  void reset();
  bool stalled(std::size_t waypointIndex, double distance, double yawError,
               double nowSeconds);

private:
  double timeoutSeconds_;
  bool tracking_ = false;
  std::size_t waypointIndex_ = 0;
  double bestDistance_ = 0.0;
  double bestYawError_ = 0.0;
  double lastProgressSeconds_ = 0.0;
};

double normalizeAngle(double angle);
double nearestForwardObstacle(const std::vector<double> &ranges,
                              double angleMin, double angleIncrement,
                              double rangeMin, double rangeMax);
StepResult follow(const Pose &pose,
                  const std::vector<contract::Waypoint> &waypoints,
                  std::size_t waypointIndex, const Velocity &previous,
                  const Limits &limits, double elapsedSeconds,
                  bool enforceIntermediateYaw = true, bool aligning = false);

}
