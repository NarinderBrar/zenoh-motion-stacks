#include <zms/manipulation/Kinematics.hpp>

#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace zms::manipulation {
namespace {

Eigen::Isometry3d toIsometry(const urdf::Pose &pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = Eigen::Quaterniond(pose.rotation.w, pose.rotation.x,
                                       pose.rotation.y, pose.rotation.z)
                        .normalized()
                        .toRotationMatrix();
  return result;
}

double squaredDistance(const std::vector<double> &a,
                       const std::vector<double> &b) {
  double total = 0.0;
  for (std::size_t index = 0; index < a.size(); ++index) {
    total += (a[index] - b[index]) * (a[index] - b[index]);
  }
  return total;
}

}

Eigen::Vector3d orientationError(const Eigen::Matrix3d &target,
                                 const Eigen::Matrix3d &current) {
  const Eigen::AngleAxisd difference(target * current.transpose());
  return difference.axis() * difference.angle();
}

KinematicChain KinematicChain::fromUrdf(const std::string &path,
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
  std::vector<ChainSegment> reversed;
  auto link = model->getLink(endEffectorFrame);
  while (link && link->name != baseFrame) {
    const auto joint = link->parent_joint;
    if (!joint) {
      throw std::invalid_argument(
          "eef_frame is not below base_frame in the URDF");
    }
    ChainSegment segment;
    segment.jointName = joint->name;
    segment.childLink = link->name;
    segment.origin = toIsometry(joint->parent_to_joint_origin_transform);
    if (joint->type == urdf::Joint::REVOLUTE ||
        joint->type == urdf::Joint::PRISMATIC) {
      const Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
      if (axis.norm() < 1e-9) {
        throw std::invalid_argument("joint " + joint->name +
                                    " has a zero axis");
      }
      segment.axis = axis.normalized();
      segment.prismatic = joint->type == urdf::Joint::PRISMATIC;
      segment.jointIndex = 0;
    } else if (joint->type != urdf::Joint::FIXED) {
      throw std::invalid_argument("unsupported joint type: " + joint->name);
    }
    reversed.push_back(segment);
    link = model->getLink(joint->parent_link_name);
  }
  if (!link) {
    throw std::invalid_argument("eef_frame is not below base_frame");
  }
  KinematicChain chain;
  chain.segments_.assign(reversed.rbegin(), reversed.rend());
  for (auto &segment : chain.segments_) {
    if (segment.jointIndex >= 0) {
      segment.jointIndex = static_cast<int>(chain.jointCount_++);
    }
  }
  if (chain.jointCount_ == 0) {
    throw std::invalid_argument("URDF chain has no controllable joints");
  }
  return chain;
}

std::size_t KinematicChain::jointCount() const { return jointCount_; }

const std::vector<ChainSegment> &KinematicChain::segments() const {
  return segments_;
}

std::vector<Eigen::Isometry3d>
KinematicChain::segmentPoses(const std::vector<double> &positions) const {
  if (positions.size() != jointCount_) {
    throw std::invalid_argument("joint position count differs from chain");
  }
  std::vector<Eigen::Isometry3d> poses;
  poses.reserve(segments_.size());
  Eigen::Isometry3d current = Eigen::Isometry3d::Identity();
  for (const auto &segment : segments_) {
    current = current * segment.origin;
    if (segment.jointIndex >= 0) {
      const double value = positions[segment.jointIndex];
      if (segment.prismatic) {
        current = current * Eigen::Translation3d(segment.axis * value);
      } else {
        current = current * Eigen::AngleAxisd(value, segment.axis);
      }
    }
    poses.push_back(current);
  }
  return poses;
}

Eigen::Isometry3d
KinematicChain::forward(const std::vector<double> &positions) const {
  return segmentPoses(positions).back();
}

Eigen::Matrix<double, 6, Eigen::Dynamic>
KinematicChain::jacobian(const std::vector<double> &positions) const {
  if (positions.size() != jointCount_) {
    throw std::invalid_argument("joint position count differs from chain");
  }
  Eigen::Matrix<double, 6, Eigen::Dynamic> result(6, jointCount_);
  std::vector<Eigen::Vector3d> axes(jointCount_);
  std::vector<Eigen::Vector3d> origins(jointCount_);
  std::vector<bool> prismatic(jointCount_, false);
  Eigen::Isometry3d current = Eigen::Isometry3d::Identity();
  for (const auto &segment : segments_) {
    current = current * segment.origin;
    if (segment.jointIndex >= 0) {
      axes[segment.jointIndex] = current.linear() * segment.axis;
      origins[segment.jointIndex] = current.translation();
      prismatic[segment.jointIndex] = segment.prismatic;
      const double value = positions[segment.jointIndex];
      if (segment.prismatic) {
        current = current * Eigen::Translation3d(segment.axis * value);
      } else {
        current = current * Eigen::AngleAxisd(value, segment.axis);
      }
    }
  }
  const Eigen::Vector3d tip = current.translation();
  for (std::size_t index = 0; index < jointCount_; ++index) {
    if (prismatic[index]) {
      result.block<3, 1>(0, index) = axes[index];
      result.block<3, 1>(3, index).setZero();
    } else {
      result.block<3, 1>(0, index) = axes[index].cross(tip - origins[index]);
      result.block<3, 1>(3, index) = axes[index];
    }
  }
  return result;
}

std::optional<std::vector<double>> KinematicChain::solveIk(
    const Eigen::Isometry3d &target, const std::vector<double> &seed,
    const std::vector<JointLimit> &limits, const IkOptions &options,
    const ConfigurationFilter &accept) const {
  if (seed.size() != jointCount_ || limits.size() != jointCount_) {
    throw std::invalid_argument("IK seed and limits must match the chain");
  }
  std::mt19937 random(options.seed);
  std::vector<std::vector<double>> solutions;
  const double lambdaSquared = options.damping * options.damping;
  for (int attempt = 0;
       attempt < options.attempts &&
       static_cast<int>(solutions.size()) < options.maxSolutions;
       ++attempt) {
    std::vector<double> q(jointCount_);
    for (std::size_t index = 0; index < jointCount_; ++index) {
      if (attempt == 0) {
        q[index] =
            std::clamp(seed[index], limits[index].lower, limits[index].upper);
      } else {
        std::uniform_real_distribution<double> range(limits[index].lower,
                                                     limits[index].upper);
        q[index] = range(random);
      }
    }
    bool converged = false;
    for (int iteration = 0; iteration < options.maxIterations; ++iteration) {
      const Eigen::Isometry3d current = forward(q);
      Eigen::Matrix<double, 6, 1> error;
      error.head<3>() = target.translation() - current.translation();
      error.tail<3>() = orientationError(target.linear(), current.linear());
      if (error.head<3>().norm() <= options.positionTolerance &&
          error.tail<3>().norm() <= options.orientationTolerance) {
        converged = true;
        break;
      }
      const auto j = jacobian(q);
      const Eigen::Matrix<double, 6, 6> jjt =
          j * j.transpose() +
          lambdaSquared * Eigen::Matrix<double, 6, 6>::Identity();
      Eigen::VectorXd step = j.transpose() * jjt.ldlt().solve(error);
      const double largest = step.cwiseAbs().maxCoeff();
      if (largest > options.maxStep) {
        step *= options.maxStep / largest;
      }
      for (std::size_t index = 0; index < jointCount_; ++index) {
        q[index] = std::clamp(q[index] + step[index], limits[index].lower,
                              limits[index].upper);
      }
    }
    if (converged && (!accept || accept(q))) {
      solutions.push_back(q);
    }
  }
  if (solutions.empty()) {
    return std::nullopt;
  }
  return *std::min_element(
      solutions.begin(), solutions.end(), [&](const auto &a, const auto &b) {
        return squaredDistance(a, seed) < squaredDistance(b, seed);
      });
}

}
