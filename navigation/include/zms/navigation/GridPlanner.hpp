#pragma once

#include <zms/contract/Contract.hpp>
#include <zms/navigation/Navigation.hpp>

#include <cstdint>
#include <stop_token>
#include <vector>

namespace zms::navigation {

struct OccupancyGrid {
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  Pose origin;
  std::vector<std::int8_t> cells;
};

OccupancyGrid parseGrid(const nlohmann::json &payload);
std::vector<contract::Waypoint> planGrid(const OccupancyGrid &grid,
                                         const Pose &start,
                                         const contract::Waypoint &goal,
                                         double inflationRadius,
                                         std::stop_token stop = {});

}
