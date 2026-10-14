#include <zms/contract/Contract.hpp>

#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace zms::contract {
namespace {

void requireFields(const nlohmann::json &value,
                   const std::unordered_set<std::string> &allowed) {
  if (!value.is_object()) {
    throw std::invalid_argument("payload must be a JSON object");
  }
  for (auto it = value.begin(); it != value.end(); ++it) {
    if (!allowed.contains(it.key())) {
      throw std::invalid_argument("unknown field: " + it.key());
    }
  }
}

std::string identifier(const nlohmann::json &value, std::string_view field) {
  const std::string name(field);
  if (!value.contains(name) || !value.at(name).is_string()) {
    throw std::invalid_argument(name + " must be a nonempty string");
  }
  const auto result = value.at(name).get<std::string>();
  if (result.empty() || result.size() > 128) {
    throw std::invalid_argument(name + " must contain 1 to 128 characters");
  }
  return result;
}

double finiteNumber(const nlohmann::json &value, std::string_view field) {
  const std::string name(field);
  if (!value.contains(name) || !value.at(name).is_number()) {
    throw std::invalid_argument(name + " must be a number");
  }
  const double result = value.at(name).get<double>();
  if (!std::isfinite(result)) {
    throw std::invalid_argument(name + " must be finite");
  }
  return result;
}

void requireVersion(const nlohmann::json &value) {
  if (!value.contains("contract") ||
      !value.at("contract").is_number_integer() ||
      value.at("contract").get<int>() != majorVersion) {
    throw std::invalid_argument("unsupported contract major version");
  }
}

}

std::string key(std::string_view prefix, std::string_view suffix) {
  if (suffix.empty() || suffix.front() == '/' || suffix.back() == '/' ||
      suffix.find("//") != std::string_view::npos ||
      suffix.find('*') != std::string_view::npos) {
    throw std::invalid_argument("invalid key suffix");
  }
  std::string result(prefix);
  while (!result.empty() && result.back() == '/') {
    result.pop_back();
  }
  if (!result.empty()) {
    if (result.front() == '/' || result.find("//") != std::string::npos ||
        result.find('*') != std::string::npos) {
      throw std::invalid_argument("invalid robot prefix");
    }
    result += '/';
  }
  result += suffix;
  return result;
}

std::string_view name(StackKind kind) {
  return kind == StackKind::Navigation ? "navigation" : "manipulation";
}

std::string_view name(CommandState state) {
  switch (state) {
  case CommandState::Pending:
    return "pending";
  case CommandState::Active:
    return "active";
  case CommandState::Succeeded:
    return "succeeded";
  case CommandState::Cancelled:
    return "cancelled";
  case CommandState::Failed:
    return "failed";
  }
  throw std::invalid_argument("unknown command state");
}

Goal parseGoal(const nlohmann::json &payload) {
  requireFields(payload, {"contract", "command_id", "client_id", "waypoints"});
  requireVersion(payload);
  Goal goal{
      identifier(payload, "command_id"), identifier(payload, "client_id"), {}};
  if (!payload.contains("waypoints") || !payload.at("waypoints").is_array() ||
      payload.at("waypoints").empty() || payload.at("waypoints").size() > 256) {
    throw std::invalid_argument("waypoints must contain 1 to 256 poses");
  }
  for (const auto &item : payload.at("waypoints")) {
    requireFields(item, {"x", "y", "yaw"});
    goal.waypoints.push_back({finiteNumber(item, "x"), finiteNumber(item, "y"),
                              finiteNumber(item, "yaw")});
  }
  return goal;
}

Cancel parseCancel(const nlohmann::json &payload) {
  requireFields(payload, {"contract", "command_id", "client_id"});
  requireVersion(payload);
  return {identifier(payload, "command_id"), identifier(payload, "client_id")};
}

Target parseTarget(const nlohmann::json &payload) {
  requireFields(payload, {"contract", "command_id", "client_id", "pose",
                          "joints", "base_frame", "eef_frame", "cartesian"});
  requireVersion(payload);
  Target target{identifier(payload, "command_id"),
                identifier(payload, "client_id"), PoseTarget{}};
  const bool hasPose = payload.contains("pose");
  const bool hasJoints = payload.contains("joints");
  if (hasPose == hasJoints) {
    throw std::invalid_argument("exactly one of pose or joints is required");
  }
  if (hasPose) {
    const auto &pose = payload.at("pose");
    requireFields(pose, {"x", "y", "z", "qx", "qy", "qz", "qw"});
    PoseTarget value{finiteNumber(pose, "x"),  finiteNumber(pose, "y"),
                     finiteNumber(pose, "z"),  finiteNumber(pose, "qx"),
                     finiteNumber(pose, "qy"), finiteNumber(pose, "qz"),
                     finiteNumber(pose, "qw")};
    const double norm = std::sqrt(value.qx * value.qx + value.qy * value.qy +
                                  value.qz * value.qz + value.qw * value.qw);
    if (std::abs(norm - 1.0) > 0.001) {
      throw std::invalid_argument("pose quaternion must have unit length");
    }
    if (payload.contains("base_frame")) {
      value.baseFrame = identifier(payload, "base_frame");
    }
    if (payload.contains("eef_frame")) {
      value.endEffectorFrame = identifier(payload, "eef_frame");
    }
    if (payload.contains("cartesian")) {
      if (!payload.at("cartesian").is_boolean()) {
        throw std::invalid_argument("cartesian must be a boolean");
      }
      value.cartesian = payload.at("cartesian").get<bool>();
    }
    target.value = std::move(value);
    return target;
  }
  if (payload.contains("base_frame") || payload.contains("eef_frame") ||
      payload.contains("cartesian")) {
    throw std::invalid_argument(
        "frame overrides and cartesian require a pose target");
  }
  const auto &joints = payload.at("joints");
  requireFields(joints, {"name", "position"});
  if (!joints.contains("name") || !joints.at("name").is_array() ||
      !joints.contains("position") || !joints.at("position").is_array() ||
      joints.at("name").empty() || joints.at("name").size() > 64 ||
      joints.at("name").size() != joints.at("position").size()) {
    throw std::invalid_argument(
        "joints require 1 to 64 matching names and positions");
  }
  JointTarget value;
  std::unordered_set<std::string> seen;
  for (std::size_t index = 0; index < joints.at("name").size(); ++index) {
    if (!joints.at("name").at(index).is_string()) {
      throw std::invalid_argument("joint name must be a string");
    }
    const auto name = joints.at("name").at(index).get<std::string>();
    if (name.empty() || name.size() > 128 || !seen.insert(name).second) {
      throw std::invalid_argument("joint names must be unique and nonempty");
    }
    if (!joints.at("position").at(index).is_number()) {
      throw std::invalid_argument("joint position must be a number");
    }
    const double position = joints.at("position").at(index).get<double>();
    if (!std::isfinite(position)) {
      throw std::invalid_argument("joint position must be finite");
    }
    value.names.push_back(name);
    value.positions.push_back(position);
  }
  target.value = std::move(value);
  return target;
}

nlohmann::json toJson(const Goal &goal) {
  nlohmann::json waypoints = nlohmann::json::array();
  for (const auto &waypoint : goal.waypoints) {
    waypoints.push_back(
        {{"x", waypoint.x}, {"y", waypoint.y}, {"yaw", waypoint.yaw}});
  }
  nlohmann::json value = {{"contract", majorVersion},
                          {"command_id", goal.commandId},
                          {"client_id", goal.clientId},
                          {"waypoints", std::move(waypoints)}};
  parseGoal(value);
  return value;
}

nlohmann::json toJson(const Cancel &cancel) {
  nlohmann::json value = {{"contract", majorVersion},
                          {"command_id", cancel.commandId},
                          {"client_id", cancel.clientId}};
  parseCancel(value);
  return value;
}

nlohmann::json toJson(const Target &target) {
  nlohmann::json value = {{"contract", majorVersion},
                          {"command_id", target.commandId},
                          {"client_id", target.clientId}};
  if (const auto *pose = std::get_if<PoseTarget>(&target.value)) {
    value["pose"] = {{"x", pose->x},   {"y", pose->y},   {"z", pose->z},
                     {"qx", pose->qx}, {"qy", pose->qy}, {"qz", pose->qz},
                     {"qw", pose->qw}};
    if (!pose->baseFrame.empty()) {
      value["base_frame"] = pose->baseFrame;
    }
    if (!pose->endEffectorFrame.empty()) {
      value["eef_frame"] = pose->endEffectorFrame;
    }
    if (pose->cartesian) {
      value["cartesian"] = true;
    }
  } else {
    const auto &joints = std::get<JointTarget>(target.value);
    value["joints"] = {{"name", joints.names}, {"position", joints.positions}};
  }
  parseTarget(value);
  return value;
}

GripperCommand parseGripper(const nlohmann::json &payload) {
  requireFields(
      payload, {"contract", "command_id", "client_id", "command", "positions"});
  requireVersion(payload);
  GripperCommand command{identifier(payload, "command_id"),
                         identifier(payload, "client_id"),
                         {},
                         {}};
  if (!payload.contains("command") || !payload.at("command").is_string()) {
    throw std::invalid_argument("command must be open, close, or position");
  }
  command.action = payload.at("command").get<std::string>();
  if (command.action != "open" && command.action != "close" &&
      command.action != "position") {
    throw std::invalid_argument("command must be open, close, or position");
  }
  if (command.action == "position") {
    if (!payload.contains("positions") || !payload.at("positions").is_array() ||
        payload.at("positions").empty() ||
        payload.at("positions").size() > 16) {
      throw std::invalid_argument("positions must hold 1 to 16 numbers");
    }
    for (const auto &item : payload.at("positions")) {
      if (!item.is_number() || !std::isfinite(item.get<double>())) {
        throw std::invalid_argument("positions must be finite numbers");
      }
      command.positions.push_back(item.get<double>());
    }
  } else if (payload.contains("positions")) {
    throw std::invalid_argument("positions require the position command");
  }
  return command;
}

nlohmann::json toJson(const GripperCommand &command) {
  nlohmann::json value = {{"contract", majorVersion},
                          {"command_id", command.commandId},
                          {"client_id", command.clientId},
                          {"command", command.action}};
  if (command.action == "position") {
    value["positions"] = command.positions;
  }
  parseGripper(value);
  return value;
}

nlohmann::json accepted(const std::string &commandId,
                        const std::string &clientId) {
  return {{"contract", majorVersion},
          {"accepted", true},
          {"command_id", commandId},
          {"client_id", clientId}};
}

nlohmann::json rejected(const std::string &commandId,
                        const std::string &clientId, std::string_view reason,
                        std::string_view message) {
  return {{"contract", majorVersion}, {"accepted", false},
          {"command_id", commandId},  {"client_id", clientId},
          {"reason", reason},         {"message", message}};
}

nlohmann::json status(const std::string &commandId, const std::string &clientId,
                      CommandState state, std::string_view reason,
                      std::string_view message) {
  nlohmann::json value = {{"contract", majorVersion},
                          {"command_id", commandId},
                          {"client_id", clientId},
                          {"state", name(state)}};
  if (!reason.empty()) {
    value["reason"] = reason;
    value["message"] = message;
  }
  return value;
}

}
