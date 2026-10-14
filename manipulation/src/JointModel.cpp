#include <zms/manipulation/JointModel.hpp>

#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace zms::manipulation {

JointModel JointModel::fromUrdf(const std::string &path,
                                const std::string &baseFrame,
                                const std::string &endEffectorFrame) {
  const auto model = urdf::parseURDFFile(path);
  if (!model) {
    throw std::invalid_argument("urdf_path could not be parsed: " + path);
  }
  if (!model->getLink(baseFrame) || !model->getLink(endEffectorFrame) ||
      baseFrame == endEffectorFrame) {
    throw std::invalid_argument(
        "base_frame and eef_frame must name distinct URDF links");
  }
  std::vector<JointLimit> reversed;
  auto link = model->getLink(endEffectorFrame);
  while (link && link->name != baseFrame) {
    const auto joint = link->parent_joint;
    if (!joint) {
      throw std::invalid_argument(
          "eef_frame is not below base_frame in the URDF");
    }
    if (joint->type == urdf::Joint::REVOLUTE ||
        joint->type == urdf::Joint::PRISMATIC) {
      if (joint->mimic || !joint->limits ||
          !std::isfinite(joint->limits->lower) ||
          !std::isfinite(joint->limits->upper) ||
          !std::isfinite(joint->limits->velocity) ||
          joint->limits->lower >= joint->limits->upper ||
          joint->limits->velocity <= 0.0) {
        throw std::invalid_argument("joint " + joint->name +
                                    " needs finite limits and no mimic");
      }
      reversed.push_back({joint->name, joint->limits->lower,
                          joint->limits->upper, joint->limits->velocity});
    } else if (joint->type != urdf::Joint::FIXED) {
      throw std::invalid_argument("unsupported joint type: " + joint->name);
    }
    link = model->getLink(joint->parent_link_name);
  }
  if (!link || reversed.empty() || reversed.size() > 64) {
    throw std::invalid_argument(
        "URDF chain must have 1 to 64 controllable joints");
  }
  JointModel result;
  result.joints_ = {reversed.rbegin(), reversed.rend()};
  return result;
}

const std::vector<JointLimit> &JointModel::joints() const { return joints_; }

std::vector<double>
JointModel::orderedPositions(const contract::JointTarget &target) const {
  if (target.names.size() != joints_.size() ||
      target.positions.size() != joints_.size()) {
    throw std::invalid_argument(
        "joint target must contain every configured joint");
  }
  std::unordered_map<std::string, double> positions;
  for (std::size_t index = 0; index < target.names.size(); ++index) {
    if (!std::isfinite(target.positions[index]) ||
        !positions.emplace(target.names[index], target.positions[index])
             .second) {
      throw std::invalid_argument(
          "joint target has a duplicate or nonfinite value");
    }
  }
  std::vector<double> ordered;
  ordered.reserve(joints_.size());
  for (const auto &joint : joints_) {
    const auto found = positions.find(joint.name);
    if (found == positions.end()) {
      throw std::invalid_argument("joint target is missing " + joint.name);
    }
    if (found->second < joint.lower || found->second > joint.upper) {
      throw std::invalid_argument("joint target exceeds limit for " +
                                  joint.name);
    }
    ordered.push_back(found->second);
  }
  return ordered;
}

}
