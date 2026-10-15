#include <zms/contract/Contract.hpp>
#include <zms/contract/ZenohClient.hpp>

#include <zenoh.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <mutex>
#include <numbers>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t stopRequested = 0;

void requestStop(int) { stopRequested = 1; }

struct State {
  std::mutex mutex;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double linear = 0.0;
  double angular = 0.0;
  double odomX = 0.0;
  double odomY = 0.0;
  double odomYaw = 0.0;
  std::vector<std::string> jointNames;
  std::vector<double> jointPositions;
  std::vector<double> jointTargets;
  std::vector<double> jointVelocities;
  std::chrono::steady_clock::time_point lastCommand =
      std::chrono::steady_clock::now();
};

std::vector<std::string> parseJointNames(const std::string &value) {
  std::vector<std::string> names;
  std::set<std::string> seen;
  std::size_t start = 0;
  while (start < value.size()) {
    const auto end = value.find(',', start);
    const auto name = value.substr(start, end - start);
    if (name.empty() || !seen.insert(name).second || names.size() >= 64) {
      throw std::invalid_argument("joint names must be unique and nonempty");
    }
    names.push_back(name);
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  if (names.empty() || value.back() == ',') {
    throw std::invalid_argument("at least one joint name is required");
  }
  return names;
}

struct Obstacle {
  double x = 0.0;
  double y = 0.0;
  double radius = 0.0;
};

Obstacle parseObstacle(const std::string &value) {
  Obstacle obstacle;
  char first = 0;
  char second = 0;
  std::istringstream input(value);
  if (!(input >> obstacle.x >> first >> obstacle.y >> second >>
        obstacle.radius) ||
      first != ',' || second != ',' || !std::isfinite(obstacle.x) ||
      !std::isfinite(obstacle.y) || !std::isfinite(obstacle.radius) ||
      obstacle.radius <= 0.0) {
    throw std::invalid_argument("--obstacle expects x,y,radius");
  }
  return obstacle;
}

struct Wall {
  double x1 = 0.0;
  double y1 = 0.0;
  double x2 = 0.0;
  double y2 = 0.0;
};

std::vector<double> parseNumbers(const std::string &value, std::size_t count,
                                 const char *option) {
  std::vector<double> numbers;
  std::istringstream input(value);
  std::string item;
  while (std::getline(input, item, ',')) {
    std::size_t consumed = 0;
    const double number = std::stod(item, &consumed);
    if (consumed != item.size() || !std::isfinite(number)) {
      throw std::invalid_argument(std::string(option) + " has a bad number");
    }
    numbers.push_back(number);
  }
  if (numbers.size() != count) {
    throw std::invalid_argument(std::string(option) + " expects " +
                                std::to_string(count) + " numbers");
  }
  return numbers;
}

double castRay(double x, double y, double angle,
               const std::vector<Obstacle> &obstacles,
               const std::vector<Wall> &walls, double maximum) {
  const double dx = std::cos(angle);
  const double dy = std::sin(angle);
  double nearest = maximum;
  for (const auto &wall : walls) {
    const double ex = wall.x2 - wall.x1;
    const double ey = wall.y2 - wall.y1;
    const double denominator = dx * ey - dy * ex;
    if (std::abs(denominator) < 1e-12) {
      continue;
    }
    const double wx = wall.x1 - x;
    const double wy = wall.y1 - y;
    const double along = (wx * ey - wy * ex) / denominator;
    const double onWall = (wx * dy - wy * dx) / denominator;
    if (along > 0.0 && onWall >= 0.0 && onWall <= 1.0) {
      nearest = std::min(nearest, along);
    }
  }
  for (const auto &obstacle : obstacles) {
    const double ox = obstacle.x - x;
    const double oy = obstacle.y - y;
    const double along = ox * dx + oy * dy;
    const double offsetSquared = ox * ox + oy * oy - along * along;
    const double radiusSquared = obstacle.radius * obstacle.radius;
    if (along <= 0.0 || offsetSquared > radiusSquared) {
      continue;
    }
    const double hit = along - std::sqrt(radiusSquared - offsetSquared);
    if (hit > 0.0) {
      nearest = std::min(nearest, hit);
    }
  }
  return nearest;
}

}

int main(int argc, char **argv) {
  try {
    if (argc < 2) {
      throw std::invalid_argument(
          "usage: zms_mock_robot <robot-prefix> [--joints name1,name2,...] "
          "[--obstacle x,y,radius]... [--wall x1,y1,x2,y2]... "
          "[--start x,y,yaw] [--odom-scale k] [--odom-yaw-drift rad_per_s]");
    }
    const std::string prefix = argv[1];
    std::string jointList;
    std::vector<Obstacle> obstacles;
    std::vector<Wall> walls;
    double startX = 0.0;
    double startY = 0.0;
    double startYaw = 0.0;
    double odomScale = 1.0;
    double odomYawDrift = 0.0;
    for (int index = 2; index < argc; index += 2) {
      if (index + 1 >= argc) {
        throw std::invalid_argument("missing option value");
      }
      const std::string_view option = argv[index];
      if (option == "--joints") {
        jointList = argv[index + 1];
      } else if (option == "--obstacle") {
        obstacles.push_back(parseObstacle(argv[index + 1]));
      } else if (option == "--wall") {
        const auto values = parseNumbers(argv[index + 1], 4, "--wall");
        walls.push_back({values[0], values[1], values[2], values[3]});
      } else if (option == "--start") {
        const auto values = parseNumbers(argv[index + 1], 3, "--start");
        startX = values[0];
        startY = values[1];
        startYaw = values[2];
      } else if (option == "--odom-scale") {
        odomScale = parseNumbers(argv[index + 1], 1, "--odom-scale")[0];
      } else if (option == "--odom-yaw-drift") {
        odomYawDrift = parseNumbers(argv[index + 1], 1, "--odom-yaw-drift")[0];
      } else {
        throw std::invalid_argument("unknown option");
      }
    }
    const auto velocityKey = zms::contract::key(prefix, "cmd_vel");
    const auto odomKey = zms::contract::key(prefix, "odom");
    const auto jointCommandKey = zms::contract::key(prefix, "cmd_joints");
    const auto jointStateKey = zms::contract::key(prefix, "joint_states");
    const auto scanKey = zms::contract::key(prefix, "scan");
    const auto truthKey = zms::contract::key(prefix, "ground_truth");
    zenoh::Session session(zms::contract::sessionConfig());
    State state;
    state.x = startX;
    state.y = startY;
    state.yaw = startYaw;
    if (!jointList.empty()) {
      state.jointNames = parseJointNames(jointList);
      state.jointPositions.resize(state.jointNames.size());
      state.jointTargets.resize(state.jointNames.size());
      state.jointVelocities.resize(state.jointNames.size());
    }
    auto subscriber = session.declare_subscriber(
        zenoh::KeyExpr(velocityKey),
        [&](const zenoh::Sample &sample) {
          try {
            if (sample.get_payload().size() > 4096) {
              return;
            }
            const auto command =
                nlohmann::json::parse(sample.get_payload().as_string());
            const double linear = command.at("linear").get<double>();
            const double angular = command.at("angular").get<double>();
            if (!std::isfinite(linear) || !std::isfinite(angular)) {
              return;
            }
            std::lock_guard lock(state.mutex);
            state.linear = std::clamp(linear, -2.0, 2.0);
            state.angular = std::clamp(angular, -4.0, 4.0);
            state.lastCommand = std::chrono::steady_clock::now();
          } catch (const std::exception &) {
          }
        },
        [] {});
    auto jointSubscriber = session.declare_subscriber(
        zenoh::KeyExpr(jointCommandKey),
        [&](const zenoh::Sample &sample) {
          try {
            if (sample.get_payload().size() > 65536) {
              return;
            }
            const auto command =
                nlohmann::json::parse(sample.get_payload().as_string());
            if (!command.contains("name") || !command.at("name").is_array() ||
                !command.contains("position") ||
                !command.at("position").is_array() ||
                command.at("name").size() != command.at("position").size()) {
              return;
            }
            std::lock_guard lock(state.mutex);
            std::vector<std::pair<std::size_t, double>> updates;
            std::set<std::string> seen;
            for (std::size_t index = 0; index < command.at("name").size();
                 ++index) {
              const auto name = command.at("name").at(index).get<std::string>();
              const double position =
                  command.at("position").at(index).get<double>();
              if (!std::isfinite(position) || !seen.insert(name).second) {
                return;
              }
              const auto found = std::find(state.jointNames.begin(),
                                           state.jointNames.end(), name);
              if (found == state.jointNames.end()) {
                return;
              }
              updates.emplace_back(
                  std::distance(state.jointNames.begin(), found), position);
            }
            for (const auto &[index, position] : updates) {
              state.jointTargets[index] = position;
            }
          } catch (const std::exception &) {
          }
        },
        [] {});
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    auto previous = std::chrono::steady_clock::now();
    unsigned tick = 0;
    while (!stopRequested) {
      const auto now = std::chrono::steady_clock::now();
      const double dt = std::chrono::duration<double>(now - previous).count();
      previous = now;
      nlohmann::json odom;
      nlohmann::json jointStates;
      nlohmann::json truth;
      {
        std::lock_guard lock(state.mutex);
        if (std::chrono::duration<double>(now - state.lastCommand).count() >
            0.5) {
          state.linear = 0.0;
          state.angular = 0.0;
        }
        state.x += std::cos(state.yaw) * state.linear * dt;
        state.y += std::sin(state.yaw) * state.linear * dt;
        state.yaw = std::remainder(state.yaw + state.angular * dt,
                                   2.0 * std::numbers::pi);
        const bool moving = state.linear != 0.0 || state.angular != 0.0;
        state.odomX += std::cos(state.odomYaw) * state.linear * odomScale * dt;
        state.odomY += std::sin(state.odomYaw) * state.linear * odomScale * dt;
        state.odomYaw = std::remainder(
            state.odomYaw +
                (state.angular + (moving ? odomYawDrift : 0.0)) * dt,
            2.0 * std::numbers::pi);
        odom = {{"contract", 1},          {"x", state.odomX},
                {"y", state.odomY},       {"yaw", state.odomYaw},
                {"linear", state.linear}, {"angular", state.angular}};
        truth = {{"contract", 1},
                 {"x", state.x},
                 {"y", state.y},
                 {"yaw", state.yaw}};
        for (std::size_t index = 0; index < state.jointNames.size(); ++index) {
          const double delta =
              state.jointTargets[index] - state.jointPositions[index];
          const double movement = std::clamp(delta, -dt, dt);
          state.jointPositions[index] += movement;
          state.jointVelocities[index] = dt > 0.0 ? movement / dt : 0.0;
        }
        jointStates = {{"contract", 1},
                       {"name", state.jointNames},
                       {"position", state.jointPositions},
                       {"velocity", state.jointVelocities}};
      }
      auto options = zenoh::Session::PutOptions::create_default();
      options.encoding = zenoh::Encoding("application/json");
      session.put(zenoh::KeyExpr(odomKey), zenoh::Bytes(odom.dump()),
                  std::move(options));
      session.put(zenoh::KeyExpr(truthKey), zenoh::Bytes(truth.dump()));
      if (!state.jointNames.empty()) {
        auto jointOptions = zenoh::Session::PutOptions::create_default();
        jointOptions.encoding = zenoh::Encoding("application/json");
        session.put(zenoh::KeyExpr(jointStateKey),
                    zenoh::Bytes(jointStates.dump()), std::move(jointOptions));
      }
      if ((!obstacles.empty() || !walls.empty()) && ++tick % 2 == 0) {
        constexpr int beams = 360;
        constexpr double maximumRange = 8.0;
        nlohmann::json ranges = nlohmann::json::array();
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        {
          std::lock_guard lock(state.mutex);
          x = state.x;
          y = state.y;
          yaw = state.yaw;
        }
        for (int beam = 0; beam < beams; ++beam) {
          const double angle =
              -std::numbers::pi + beam * 2.0 * std::numbers::pi / beams;
          const double range =
              castRay(x, y, yaw + angle, obstacles, walls, maximumRange);
          if (range >= maximumRange) {
            ranges.push_back(nullptr);
          } else {
            ranges.push_back(range);
          }
        }
        auto scanOptions = zenoh::Session::PutOptions::create_default();
        scanOptions.encoding = zenoh::Encoding("application/json");
        session.put(zenoh::KeyExpr(scanKey),
                    zenoh::Bytes(nlohmann::json{
                        {"contract", 1},
                        {"angle_min", -std::numbers::pi},
                        {"angle_increment", 2.0 * std::numbers::pi / beams},
                        {"range_min", 0.05},
                        {"range_max", maximumRange},
                        {"ranges",
                         ranges}}.dump()),
                    std::move(scanOptions));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
