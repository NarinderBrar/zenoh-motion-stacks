#include <zms/navigation/Visualization.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

namespace zms::navigation {

nlohmann::json posesToJson(const std::vector<contract::Waypoint> &poses) {
  nlohmann::json array = nlohmann::json::array();
  for (const auto &pose : poses) {
    array.push_back({{"x", pose.x}, {"y", pose.y}, {"yaw", pose.yaw}});
  }
  return array;
}

std::vector<contract::Waypoint>
pathFromCurrentPose(const Pose &currentPose,
                    const std::vector<contract::Waypoint> &poses) {
  std::vector<contract::Waypoint> path;
  path.reserve(poses.size() + 1);
  path.push_back({currentPose.x, currentPose.y, currentPose.yaw});
  path.insert(path.end(), poses.begin(), poses.end());
  return path;
}

nlohmann::json gridToJson(const OccupancyGrid &grid) {
  nlohmann::json data = nlohmann::json::array();
  for (const auto cell : grid.cells) {
    data.push_back(static_cast<int>(cell));
  }
  return {{"contract", 1},
          {"width", grid.width},
          {"height", grid.height},
          {"resolution", grid.resolution},
          {"origin",
           {{"x", grid.origin.x},
            {"y", grid.origin.y},
            {"yaw", grid.origin.yaw}}},
          {"data", std::move(data)}};
}

OccupancyGrid inflateGrid(const OccupancyGrid &grid, double radius) {
  OccupancyGrid inflated = grid;
  const int cells = static_cast<int>(std::ceil(radius / grid.resolution));
  const int span = std::min(cells, 100);
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const auto value =
          grid.cells[static_cast<std::size_t>(y) * grid.width + x];
      if (value >= 0 && value < 50) {
        continue;
      }
      for (int offsetY = -span; offsetY <= span; ++offsetY) {
        for (int offsetX = -span; offsetX <= span; ++offsetX) {
          if (offsetX * offsetX + offsetY * offsetY > span * span) {
            continue;
          }
          const int nx = x + offsetX;
          const int ny = y + offsetY;
          if (nx >= 0 && ny >= 0 && nx < grid.width && ny < grid.height) {
            inflated.cells[static_cast<std::size_t>(ny) * grid.width + nx] =
                100;
          }
        }
      }
    }
  }
  return inflated;
}

void markPoints(OccupancyGrid &grid, const std::vector<Point2> &points) {
  const double cosine = std::cos(grid.origin.yaw);
  const double sine = std::sin(grid.origin.yaw);
  for (const auto &point : points) {
    const double dx = point.x - grid.origin.x;
    const double dy = point.y - grid.origin.y;
    const double gx = (cosine * dx + sine * dy) / grid.resolution;
    const double gy = (-sine * dx + cosine * dy) / grid.resolution;
    const int x = static_cast<int>(std::floor(gx));
    const int y = static_cast<int>(std::floor(gy));
    if (x >= 0 && y >= 0 && x < grid.width && y < grid.height) {
      grid.cells[static_cast<std::size_t>(y) * grid.width + x] = 100;
    }
  }
}

OccupancyGrid downsampleGrid(const OccupancyGrid &grid, int maximumCells,
                             int &strideOut) {
  strideOut = 1;
  while (static_cast<std::int64_t>(grid.width / strideOut + 1) *
             (grid.height / strideOut + 1) >
         maximumCells) {
    ++strideOut;
  }
  if (strideOut == 1) {
    return grid;
  }
  OccupancyGrid reduced;
  reduced.resolution = grid.resolution * strideOut;
  reduced.origin = grid.origin;
  reduced.width = (grid.width + strideOut - 1) / strideOut;
  reduced.height = (grid.height + strideOut - 1) / strideOut;
  reduced.cells.assign(static_cast<std::size_t>(reduced.width * reduced.height),
                       0);
  for (int y = 0; y < reduced.height; ++y) {
    for (int x = 0; x < reduced.width; ++x) {
      std::int8_t value = 0;
      for (int oy = 0; oy < strideOut; ++oy) {
        for (int ox = 0; ox < strideOut; ++ox) {
          const int sx = x * strideOut + ox;
          const int sy = y * strideOut + oy;
          if (sx < grid.width && sy < grid.height) {
            const auto cell =
                grid.cells[static_cast<std::size_t>(sy) * grid.width + sx];
            if (cell < 0) {
              value = -1;
            } else if (cell >= 50 && value >= 0) {
              value = 100;
            }
          }
        }
      }
      reduced.cells[static_cast<std::size_t>(y) * reduced.width + x] = value;
    }
  }
  return reduced;
}

bool sameGrid(const OccupancyGrid &a, const OccupancyGrid &b) {
  return a.width == b.width && a.height == b.height &&
         a.resolution == b.resolution && a.origin.x == b.origin.x &&
         a.origin.y == b.origin.y && a.origin.yaw == b.origin.yaw &&
         a.cells == b.cells;
}

OccupancyGrid buildLocalWindowGrid(const Pose &pose, double windowMeters,
                                   double resolution, double inflationRadius,
                                   const std::vector<Point2> &obstacles) {
  OccupancyGrid grid;
  grid.width = static_cast<int>(std::ceil(windowMeters / resolution));
  grid.height = grid.width;
  grid.resolution = resolution;
  const double half = windowMeters / 2.0;
  grid.origin.x = pose.x - half * (std::cos(pose.yaw) - std::sin(pose.yaw));
  grid.origin.y = pose.y - half * (std::sin(pose.yaw) + std::cos(pose.yaw));
  grid.origin.yaw = pose.yaw;
  grid.cells.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
  markPoints(grid, obstacles);
  return inflateGrid(grid, inflationRadius);
}

}
