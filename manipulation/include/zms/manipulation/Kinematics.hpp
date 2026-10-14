#pragma once

#include <zms/manipulation/JointModel.hpp>

#include <Eigen/Geometry>

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace zms::manipulation {

struct ChainSegment {
  std::string jointName;
  std::string childLink;
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  int jointIndex = -1;
  bool prismatic = false;
};

struct IkOptions {
  double positionTolerance = 0.002;
  double orientationTolerance = 0.01;
  int maxIterations = 300;
  int attempts = 32;
  int maxSolutions = 4;
  double damping = 0.05;
  double maxStep = 0.2;
  unsigned seed = 1;
};

using ConfigurationFilter = std::function<bool(const std::vector<double> &)>;

class KinematicChain {
public:
  static KinematicChain fromUrdf(const std::string &path,
                                 const std::string &baseFrame,
                                 const std::string &endEffectorFrame);

  std::size_t jointCount() const;
  const std::vector<ChainSegment> &segments() const;
  Eigen::Isometry3d forward(const std::vector<double> &positions) const;
  std::vector<Eigen::Isometry3d>
  segmentPoses(const std::vector<double> &positions) const;
  Eigen::Matrix<double, 6, Eigen::Dynamic>
  jacobian(const std::vector<double> &positions) const;
  std::optional<std::vector<double>>
  solveIk(const Eigen::Isometry3d &target, const std::vector<double> &seed,
          const std::vector<JointLimit> &limits, const IkOptions &options,
          const ConfigurationFilter &accept = {}) const;

private:
  std::vector<ChainSegment> segments_;
  std::size_t jointCount_ = 0;
};

Eigen::Vector3d orientationError(const Eigen::Matrix3d &target,
                                 const Eigen::Matrix3d &current);

}
