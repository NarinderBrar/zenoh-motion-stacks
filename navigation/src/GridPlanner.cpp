#include <zms/navigation/GridPlanner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace zms::navigation {
namespace {

constexpr int maximumCells = 1000000;

struct Cell {
  int x;
  int y;
};

int indexOf(const OccupancyGrid &grid, int x, int y) {
  return y * grid.width + x;
}

bool inside(const OccupancyGrid &grid, int x, int y) {
  return x >= 0 && y >= 0 && x < grid.width && y < grid.height;
}

Cell worldToCell(const OccupancyGrid &grid, double x, double y) {
  const double dx = x - grid.origin.x;
  const double dy = y - grid.origin.y;
  const double cosine = std::cos(grid.origin.yaw);
  const double sine = std::sin(grid.origin.yaw);
  const double cellX = std::floor((cosine * dx + sine * dy) / grid.resolution);
  const double cellY = std::floor((-sine * dx + cosine * dy) / grid.resolution);
  if (!std::isfinite(cellX) || !std::isfinite(cellY) || cellX < 0.0 ||
      cellY < 0.0 || cellX >= grid.width || cellY >= grid.height) {
    throw std::invalid_argument("point lies outside the map");
  }
  return {static_cast<int>(cellX), static_cast<int>(cellY)};
}

contract::Waypoint cellToWorld(const OccupancyGrid &grid, Cell cell) {
  const double x = (cell.x + 0.5) * grid.resolution;
  const double y = (cell.y + 0.5) * grid.resolution;
  const double cosine = std::cos(grid.origin.yaw);
  const double sine = std::sin(grid.origin.yaw);
  return {grid.origin.x + cosine * x - sine * y,
          grid.origin.y + sine * x + cosine * y, 0.0};
}

std::vector<bool> inflatedObstacles(const OccupancyGrid &grid, double radius,
                                    std::stop_token stop) {
  const int cells = static_cast<int>(std::ceil(radius / grid.resolution));
  if (cells > 100) {
    throw std::invalid_argument("inflation radius exceeds 100 cells");
  }
  std::vector<bool> blocked(grid.cells.size(), false);
  for (int y = 0; y < grid.height; ++y) {
    if (stop.stop_requested()) {
      throw std::runtime_error("planning cancelled");
    }
    for (int x = 0; x < grid.width; ++x) {
      const auto value = grid.cells[indexOf(grid, x, y)];
      if (value >= 0 && value < 50) {
        continue;
      }
      for (int offsetY = -cells; offsetY <= cells; ++offsetY) {
        for (int offsetX = -cells; offsetX <= cells; ++offsetX) {
          if (offsetX * offsetX + offsetY * offsetY > cells * cells) {
            continue;
          }
          const int neighborX = x + offsetX;
          const int neighborY = y + offsetY;
          if (inside(grid, neighborX, neighborY)) {
            blocked[indexOf(grid, neighborX, neighborY)] = true;
          }
        }
      }
    }
  }
  return blocked;
}

}

OccupancyGrid parseGrid(const nlohmann::json &payload) {
  if (!payload.is_object() || !payload.contains("contract") ||
      payload.at("contract") != contract::majorVersion ||
      !payload.contains("width") || !payload.at("width").is_number_integer() ||
      !payload.contains("height") ||
      !payload.at("height").is_number_integer() ||
      !payload.contains("resolution") ||
      !payload.at("resolution").is_number() || !payload.contains("origin") ||
      !payload.at("origin").is_object() || !payload.contains("data") ||
      !payload.at("data").is_array()) {
    throw std::invalid_argument("invalid occupancy grid fields");
  }
  OccupancyGrid grid;
  grid.width = payload.at("width").get<int>();
  grid.height = payload.at("height").get<int>();
  grid.resolution = payload.at("resolution").get<double>();
  if (grid.width < 1 || grid.height < 1 ||
      static_cast<std::int64_t>(grid.width) * grid.height > maximumCells ||
      !std::isfinite(grid.resolution) || grid.resolution <= 0.0) {
    throw std::invalid_argument("invalid occupancy grid dimensions");
  }
  const auto &origin = payload.at("origin");
  if (!origin.contains("x") || !origin.at("x").is_number() ||
      !origin.contains("y") || !origin.at("y").is_number() ||
      !origin.contains("yaw") || !origin.at("yaw").is_number()) {
    throw std::invalid_argument("invalid occupancy grid origin");
  }
  grid.origin = {origin.at("x").get<double>(), origin.at("y").get<double>(),
                 origin.at("yaw").get<double>()};
  if (!std::isfinite(grid.origin.x) || !std::isfinite(grid.origin.y) ||
      !std::isfinite(grid.origin.yaw)) {
    throw std::invalid_argument("nonfinite occupancy grid origin");
  }
  if (payload.at("data").size() !=
      static_cast<std::size_t>(grid.width * grid.height)) {
    throw std::invalid_argument("occupancy grid data length differs from size");
  }
  grid.cells.reserve(payload.at("data").size());
  for (const auto &cell : payload.at("data")) {
    if (!cell.is_number_integer()) {
      throw std::invalid_argument("occupancy cells must be integers");
    }
    const int value = cell.get<int>();
    if (value < -1 || value > 100) {
      throw std::invalid_argument("occupancy cells must be -1 to 100");
    }
    grid.cells.push_back(static_cast<std::int8_t>(value));
  }
  return grid;
}

std::vector<contract::Waypoint> planGrid(const OccupancyGrid &grid,
                                         const Pose &start,
                                         const contract::Waypoint &goal,
                                         double inflationRadius,
                                         std::stop_token stop) {
  if (grid.width < 1 || grid.height < 1 ||
      static_cast<std::int64_t>(grid.width) * grid.height > maximumCells ||
      grid.cells.size() != static_cast<std::size_t>(grid.width * grid.height) ||
      !std::isfinite(grid.resolution) || grid.resolution <= 0.0 ||
      !std::isfinite(grid.origin.x) || !std::isfinite(grid.origin.y) ||
      !std::isfinite(grid.origin.yaw) || !std::isfinite(inflationRadius) ||
      inflationRadius < 0.0 || !std::isfinite(start.x) ||
      !std::isfinite(start.y) || !std::isfinite(goal.x) ||
      !std::isfinite(goal.y) || !std::isfinite(goal.yaw)) {
    throw std::invalid_argument("invalid grid, start, goal, or inflation");
  }
  const Cell startCell = worldToCell(grid, start.x, start.y);
  const Cell goalCell = worldToCell(grid, goal.x, goal.y);
  if (!inside(grid, startCell.x, startCell.y) ||
      !inside(grid, goalCell.x, goalCell.y)) {
    throw std::invalid_argument("start or goal lies outside the map");
  }
  const auto blocked = inflatedObstacles(grid, inflationRadius, stop);
  const int startIndex = indexOf(grid, startCell.x, startCell.y);
  const int goalIndex = indexOf(grid, goalCell.x, goalCell.y);
  if (blocked[startIndex] || blocked[goalIndex]) {
    throw std::invalid_argument("start or goal lies inside an obstacle");
  }
  struct Node {
    double cost;
    int index;
    bool operator<(const Node &other) const { return cost > other.cost; }
  };
  std::priority_queue<Node> open;
  std::vector<double> costs(grid.cells.size(),
                            std::numeric_limits<double>::infinity());
  std::vector<int> parents(grid.cells.size(), -1);
  costs[startIndex] = 0.0;
  open.push({0.0, startIndex});
  constexpr int offsets[8][2] = {{-1, 0},  {1, 0},  {0, -1}, {0, 1},
                                 {-1, -1}, {1, -1}, {-1, 1}, {1, 1}};
  while (!open.empty()) {
    if (stop.stop_requested()) {
      throw std::runtime_error("planning cancelled");
    }
    const int current = open.top().index;
    open.pop();
    if (current == goalIndex) {
      break;
    }
    const int x = current % grid.width;
    const int y = current / grid.width;
    for (const auto &offset : offsets) {
      const int nextX = x + offset[0];
      const int nextY = y + offset[1];
      if (!inside(grid, nextX, nextY) || blocked[indexOf(grid, nextX, nextY)]) {
        continue;
      }
      if (offset[0] != 0 && offset[1] != 0 &&
          (blocked[indexOf(grid, x + offset[0], y)] ||
           blocked[indexOf(grid, x, y + offset[1])])) {
        continue;
      }
      const int next = indexOf(grid, nextX, nextY);
      const double candidate =
          costs[current] +
          (offset[0] != 0 && offset[1] != 0 ? std::sqrt(2.0) : 1.0);
      if (candidate >= costs[next]) {
        continue;
      }
      costs[next] = candidate;
      parents[next] = current;
      const double dx = goalCell.x - nextX;
      const double dy = goalCell.y - nextY;
      open.push({candidate + std::hypot(dx, dy), next});
    }
  }
  if (startIndex != goalIndex && parents[goalIndex] < 0) {
    throw std::runtime_error("no path through occupancy grid");
  }
  std::vector<Cell> cells;
  for (int current = goalIndex; current != startIndex;
       current = parents[current]) {
    cells.push_back({current % grid.width, current / grid.width});
  }
  std::reverse(cells.begin(), cells.end());
  std::vector<contract::Waypoint> route;
  for (std::size_t index = 0; index < cells.size(); ++index) {
    if (index + 1 == cells.size()) {
      break;
    }
    const int dx = cells[index + 1].x - cells[index].x;
    const int dy = cells[index + 1].y - cells[index].y;
    if (index > 0 && dx == cells[index].x - cells[index - 1].x &&
        dy == cells[index].y - cells[index - 1].y) {
      continue;
    }
    route.push_back(cellToWorld(grid, cells[index]));
  }
  route.push_back(goal);
  for (std::size_t index = 0; index + 1 < route.size(); ++index) {
    route[index].yaw = std::atan2(route[index + 1].y - route[index].y,
                                  route[index + 1].x - route[index].x);
  }
  return route;
}

}
