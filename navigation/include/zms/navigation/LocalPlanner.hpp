#pragma once

#include <zms/contract/Contract.hpp>
#include <zms/navigation/Navigation.hpp>

#include <optional>
#include <vector>

namespace zms::navigation {

struct Point2 {
  double x = 0.0;
  double y = 0.0;
};

struct LocalPlannerConfig {
  double robotRadius = 0.25;
  double safetyMargin = 0.1;
  double horizonSeconds = 1.5;
  double simulationStep = 0.1;
  double windowSeconds = 0.5;
  int linearSamples = 7;
  int angularSamples = 15;
  double sensingRange = 3.0;
  double progressWeight = 1.0;
  double pathWeight = 1.0;
  double headingWeight = 0.4;
  double clearanceWeight = 0.3;
  double speedWeight = 0.2;
};

struct LocalPlan {
  bool found = false;
  Velocity target;
  Velocity velocity;
  double clearance = 0.0;
};

std::vector<Point2> scanToPoints(const std::vector<double> &ranges,
                                 double angleMin, double angleIncrement,
                                 double rangeMin, double rangeMax);

std::vector<Point2> toWorld(const std::vector<Point2> &robotPoints,
                            const Pose &pose);

double clearanceAt(const Pose &pose, const std::vector<Point2> &obstacles);

bool segmentClear(const Pose &from, const contract::Waypoint &to,
                  const std::vector<Point2> &obstacles, double clearance);

LocalPlan chooseVelocity(const Pose &pose, const Velocity &current,
                         const std::vector<Point2> &obstacles,
                         const std::vector<contract::Waypoint> &path,
                         double distanceToGoal, const Limits &limits,
                         const LocalPlannerConfig &config,
                         double controlPeriod);

std::size_t lookaheadIndex(const Pose &pose,
                           const std::vector<contract::Waypoint> &route,
                           std::size_t current, double lookahead);

std::optional<std::vector<contract::Waypoint>>
planLocalPath(const Pose &pose, const contract::Waypoint &target,
              const std::vector<Point2> &obstacles, double clearance,
              double windowSize = 6.0, double resolution = 0.05);

double freeTurnDirection(const Pose &pose,
                         const std::vector<Point2> &obstacles);

}
