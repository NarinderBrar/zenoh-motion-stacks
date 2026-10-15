#pragma once

#include <zms/contract/Contract.hpp>
#include <zms/navigation/GridPlanner.hpp>
#include <zms/navigation/LocalPlanner.hpp>
#include <zms/navigation/Navigation.hpp>

#include <nlohmann/json.hpp>

#include <vector>

namespace zms::navigation {

nlohmann::json posesToJson(const std::vector<contract::Waypoint> &poses);
std::vector<contract::Waypoint>
pathFromCurrentPose(const Pose &currentPose,
                    const std::vector<contract::Waypoint> &poses);
nlohmann::json gridToJson(const OccupancyGrid &grid);
OccupancyGrid inflateGrid(const OccupancyGrid &grid, double radius);
void markPoints(OccupancyGrid &grid, const std::vector<Point2> &points);
OccupancyGrid downsampleGrid(const OccupancyGrid &grid, int maximumCells,
                             int &strideOut);
bool sameGrid(const OccupancyGrid &a, const OccupancyGrid &b);
OccupancyGrid buildLocalWindowGrid(const Pose &pose, double windowMeters,
                                   double resolution, double inflationRadius,
                                   const std::vector<Point2> &obstacles);

}
