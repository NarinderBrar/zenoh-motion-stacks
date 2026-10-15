#pragma once

#include <nlohmann/json.hpp>

#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace zms::contract {

inline constexpr int majorVersion = 1;

enum class StackKind { Navigation, Manipulation };
enum class CommandState { Pending, Active, Succeeded, Cancelled, Failed };

struct Waypoint {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  bool operator==(const Waypoint &) const = default;
};

struct Goal {
  std::string commandId;
  std::string clientId;
  std::vector<Waypoint> waypoints;
};

struct Cancel {
  std::string commandId;
  std::string clientId;
};

struct PoseTarget {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  std::string baseFrame;
  std::string endEffectorFrame;
  bool cartesian = false;
};

struct JointTarget {
  std::vector<std::string> names;
  std::vector<double> positions;
};

struct Target {
  std::string commandId;
  std::string clientId;
  std::variant<PoseTarget, JointTarget> value;
};

struct GripperCommand {
  std::string commandId;
  std::string clientId;
  std::string action;
  std::vector<double> positions;
};

std::string key(std::string_view prefix, std::string_view suffix);
std::string_view name(StackKind kind);
std::string_view name(CommandState state);
Goal parseGoal(const nlohmann::json &payload);
Cancel parseCancel(const nlohmann::json &payload);
Target parseTarget(const nlohmann::json &payload);
GripperCommand parseGripper(const nlohmann::json &payload);
nlohmann::json toJson(const Goal &goal);
nlohmann::json toJson(const Cancel &cancel);
nlohmann::json toJson(const Target &target);
nlohmann::json toJson(const GripperCommand &command);
nlohmann::json accepted(const std::string &commandId,
                        const std::string &clientId);
nlohmann::json rejected(const std::string &commandId,
                        const std::string &clientId, std::string_view reason,
                        std::string_view message);
nlohmann::json status(const std::string &commandId, const std::string &clientId,
                      CommandState state, std::string_view reason = {},
                      std::string_view message = {});

}
