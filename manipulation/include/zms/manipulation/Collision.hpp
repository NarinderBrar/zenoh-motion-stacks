#pragma once

#include <zms/manipulation/Kinematics.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace fcl {
template <typename S> class CollisionGeometry;
}

namespace zms::manipulation {

enum class ShapeKind { Box, Cylinder, Sphere };

struct EnvironmentObject {
  std::string name;
  ShapeKind kind = ShapeKind::Box;
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};

class CollisionModel {
public:
  static CollisionModel fromUrdf(const std::string &path,
                                 const std::string &baseFrame,
                                 const KinematicChain &chain,
                                 const std::vector<double> &reference,
                                 double padding);

  void setEnvironment(std::vector<EnvironmentObject> objects);
  bool inCollision(const std::vector<double> &positions,
                   std::string *detail = nullptr) const;
  std::size_t robotGeometryCount() const;
  std::size_t selfPairCount() const;
  std::size_t skippedMeshCount() const;

private:
  struct Geometry {
    std::string link;
    int anchor = -1;
    Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
    std::shared_ptr<fcl::CollisionGeometry<double>> shape;
  };

  std::vector<Eigen::Isometry3d>
  geometryPoses(const std::vector<double> &positions) const;

  KinematicChain chain_;
  std::vector<Geometry> geometries_;
  std::vector<std::pair<std::size_t, std::size_t>> selfPairs_;
  std::vector<EnvironmentObject> environment_;
  std::vector<std::shared_ptr<fcl::CollisionGeometry<double>>>
      environmentShapes_;
  std::size_t skippedMeshes_ = 0;
};

}
