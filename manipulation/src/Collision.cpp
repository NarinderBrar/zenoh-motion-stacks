#include <zms/manipulation/Collision.hpp>

#include <fcl/fcl.h>
#include <urdf_parser/urdf_parser.h>

#include <map>
#include <set>
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

std::shared_ptr<fcl::CollisionGeometryd>
makeShape(ShapeKind kind, const Eigen::Vector3d &size, double padding) {
  switch (kind) {
  case ShapeKind::Box:
    return std::make_shared<fcl::Boxd>(size.x() + 2.0 * padding,
                                       size.y() + 2.0 * padding,
                                       size.z() + 2.0 * padding);
  case ShapeKind::Cylinder:
    return std::make_shared<fcl::Cylinderd>(size.x() + padding,
                                            size.y() + 2.0 * padding);
  case ShapeKind::Sphere:
    return std::make_shared<fcl::Sphered>(size.x() + padding);
  }
  throw std::invalid_argument("unknown shape kind");
}

bool collide(const std::shared_ptr<fcl::CollisionGeometryd> &a,
             const Eigen::Isometry3d &poseA,
             const std::shared_ptr<fcl::CollisionGeometryd> &b,
             const Eigen::Isometry3d &poseB) {
  const fcl::CollisionObjectd first(a, poseA);
  const fcl::CollisionObjectd second(b, poseB);
  fcl::CollisionRequestd request;
  fcl::CollisionResultd result;
  fcl::collide(&first, &second, request, result);
  return result.isCollision();
}

}

CollisionModel CollisionModel::fromUrdf(const std::string &path,
                                        const std::string &baseFrame,
                                        const KinematicChain &chain,
                                        const std::vector<double> &reference,
                                        double padding) {
  if (padding < 0.0) {
    throw std::invalid_argument("collision padding must not be negative");
  }
  const auto model = urdf::parseURDFFile(path);
  if (!model || !model->getLink(baseFrame)) {
    throw std::invalid_argument("URDF or base_frame unavailable: " + path);
  }
  std::map<std::string, Eigen::Isometry3d> rootPoses;
  std::vector<urdf::LinkConstSharedPtr> pending = {model->getRoot()};
  rootPoses[model->getRoot()->name] = Eigen::Isometry3d::Identity();
  while (!pending.empty()) {
    const auto link = pending.back();
    pending.pop_back();
    for (const auto &child : link->child_links) {
      rootPoses[child->name] =
          rootPoses[link->name] *
          toIsometry(child->parent_joint->parent_to_joint_origin_transform);
      pending.push_back(child);
    }
  }
  std::map<std::string, int> segmentByLink;
  for (std::size_t index = 0; index < chain.segments().size(); ++index) {
    segmentByLink[chain.segments()[index].childLink] = static_cast<int>(index);
  }
  const Eigen::Isometry3d baseInverse = rootPoses.at(baseFrame).inverse();

  CollisionModel result;
  result.chain_ = chain;
  for (const auto &[name, rootPose] : rootPoses) {
    const auto link = model->getLink(name);
    int anchor = -1;
    for (auto current = link; current; current = current->getParent()) {
      if (const auto found = segmentByLink.find(current->name);
          found != segmentByLink.end()) {
        anchor = found->second;
        break;
      }
      if (current->name == baseFrame) {
        break;
      }
    }
    const Eigen::Isometry3d frame =
        anchor >= 0
            ? rootPoses.at(chain.segments()[anchor].childLink).inverse() *
                  rootPose
            : baseInverse * rootPose;
    for (const auto &collision : link->collision_array) {
      if (!collision || !collision->geometry) {
        continue;
      }
      Geometry geometry;
      geometry.link = name;
      geometry.anchor = anchor;
      geometry.relative = frame * toIsometry(collision->origin);
      switch (collision->geometry->type) {
      case urdf::Geometry::BOX: {
        const auto &box =
            static_cast<const urdf::Box &>(*collision->geometry).dim;
        geometry.shape = makeShape(
            ShapeKind::Box, Eigen::Vector3d(box.x, box.y, box.z), padding);
        break;
      }
      case urdf::Geometry::CYLINDER: {
        const auto &cylinder =
            static_cast<const urdf::Cylinder &>(*collision->geometry);
        geometry.shape = makeShape(
            ShapeKind::Cylinder,
            Eigen::Vector3d(cylinder.radius, cylinder.length, 0.0), padding);
        break;
      }
      case urdf::Geometry::SPHERE: {
        const auto &sphere =
            static_cast<const urdf::Sphere &>(*collision->geometry);
        geometry.shape =
            makeShape(ShapeKind::Sphere,
                      Eigen::Vector3d(sphere.radius, 0.0, 0.0), padding);
        break;
      }
      default:
        ++result.skippedMeshes_;
        continue;
      }
      result.geometries_.push_back(std::move(geometry));
    }
  }

  std::set<std::pair<std::string, std::string>> adjacent;
  for (const auto &[name, unused] : rootPoses) {
    const auto link = model->getLink(name);
    if (const auto parent = link->getParent()) {
      adjacent.insert({parent->name, name});
      adjacent.insert({name, parent->name});
    }
  }
  const auto poses = result.geometryPoses(reference);
  for (std::size_t a = 0; a < result.geometries_.size(); ++a) {
    for (std::size_t b = a + 1; b < result.geometries_.size(); ++b) {
      const auto &first = result.geometries_[a];
      const auto &second = result.geometries_[b];
      if (first.link == second.link ||
          (first.anchor < 0 && second.anchor < 0) ||
          adjacent.contains({first.link, second.link}) ||
          collide(first.shape, poses[a], second.shape, poses[b])) {
        continue;
      }
      result.selfPairs_.emplace_back(a, b);
    }
  }
  return result;
}

std::vector<Eigen::Isometry3d>
CollisionModel::geometryPoses(const std::vector<double> &positions) const {
  const auto segments = chain_.segmentPoses(positions);
  std::vector<Eigen::Isometry3d> poses;
  poses.reserve(geometries_.size());
  for (const auto &geometry : geometries_) {
    poses.push_back(geometry.anchor >= 0
                        ? segments[geometry.anchor] * geometry.relative
                        : geometry.relative);
  }
  return poses;
}

void CollisionModel::setEnvironment(std::vector<EnvironmentObject> objects) {
  std::vector<std::shared_ptr<fcl::CollisionGeometryd>> shapes;
  shapes.reserve(objects.size());
  for (const auto &object : objects) {
    if ((object.size.array() < 0.0).any()) {
      throw std::invalid_argument("environment object " + object.name +
                                  " has a negative size");
    }
    shapes.push_back(makeShape(object.kind, object.size, 0.0));
  }
  environment_ = std::move(objects);
  environmentShapes_ = std::move(shapes);
}

bool CollisionModel::inCollision(const std::vector<double> &positions,
                                 std::string *detail) const {
  const auto poses = geometryPoses(positions);
  for (const auto &[a, b] : selfPairs_) {
    if (collide(geometries_[a].shape, poses[a], geometries_[b].shape,
                poses[b])) {
      if (detail != nullptr) {
        *detail = geometries_[a].link + " collides with " + geometries_[b].link;
      }
      return true;
    }
  }
  for (std::size_t g = 0; g < geometries_.size(); ++g) {
    if (geometries_[g].anchor < 0) {
      continue;
    }
    for (std::size_t e = 0; e < environment_.size(); ++e) {
      if (collide(geometries_[g].shape, poses[g], environmentShapes_[e],
                  environment_[e].pose)) {
        if (detail != nullptr) {
          *detail =
              geometries_[g].link + " collides with " + environment_[e].name;
        }
        return true;
      }
    }
  }
  return false;
}

std::size_t CollisionModel::robotGeometryCount() const {
  return geometries_.size();
}

std::size_t CollisionModel::selfPairCount() const { return selfPairs_.size(); }

std::size_t CollisionModel::skippedMeshCount() const { return skippedMeshes_; }

}
