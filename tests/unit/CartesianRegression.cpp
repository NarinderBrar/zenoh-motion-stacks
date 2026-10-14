#include <zms/manipulation/Collision.hpp>
#include <zms/manipulation/JointModel.hpp>
#include <zms/manipulation/Kinematics.hpp>
#include <zms/manipulation/Planner.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

TEST(CartesianRegression, MovesFromMeasuredMobileManipulatorPose) {
  const char *urdf = "tests/data/Mobile_manipulator.urdf";
  const auto chain = zms::manipulation::KinematicChain::fromUrdf(
      urdf, "base_link", "tool_flange");
  const auto model =
      zms::manipulation::JointModel::fromUrdf(urdf, "base_link", "tool_flange");
  const auto measured = model.measuredPositions(
      {"left_wheel_joint", "forearm_joint", "left_finger_joint",
       "wrist_pitch_joint", "arm_base_joint", "tool_flange_joint",
       "upper_arm_joint", "right_finger_joint", "wrist_roll_joint"},
      {0.0, 0.51, 0.035, 0.409, 0.0, 0.0, 0.31, 0.035, 0.0});
  ASSERT_TRUE(measured.has_value());
  const auto startPose = chain.forward(*measured);
  EXPECT_NEAR(startPose.translation().x(), 0.386, 0.003);
  EXPECT_NEAR(startPose.translation().z(), 0.23, 0.003);

  auto collision = zms::manipulation::CollisionModel::fromUrdf(
      urdf, "base_link", chain, std::vector<double>(6, 0.0), 0.01);
  zms::manipulation::EnvironmentObject table;
  table.name = "table";
  table.size = Eigen::Vector3d(0.5, 0.8, 0.05);
  table.pose.translation() = Eigen::Vector3d(0.85, 0.0, 0.3);
  collision.setEnvironment({table});
  const auto valid = [&](const std::vector<double> &q) {
    return !collision.inCollision(q);
  };
  ASSERT_TRUE(valid(*measured));
  auto doubledPadding = zms::manipulation::CollisionModel::fromUrdf(
      urdf, "base_link", chain, std::vector<double>(6, 0.0), 0.02);
  doubledPadding.setEnvironment({table});
  std::string collisionDetail;
  EXPECT_TRUE(doubledPadding.inCollision(*measured, &collisionDetail));
  EXPECT_NE(collisionDetail.find("lidar_link"), std::string::npos);

  const std::array<Eigen::Vector3d, 3> positions = {
      Eigen::Vector3d(0.386, 0.0, 0.28), Eigen::Vector3d(0.43, 0.0, 0.23),
      Eigen::Vector3d(0.47, 0.0, 0.33)};
  for (std::size_t index = 0; index < positions.size(); ++index) {
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = positions[index];
    const double qy = index == 2 ? 0.5646 : 0.5765;
    const double qw = index == 2 ? 0.8253 : 0.8171;
    target.linear() =
        Eigen::Quaterniond(qw, 0.0, qy, 0.0).normalized().toRotationMatrix();
    if (index == 0) {
      const auto oldValidity = [&](const std::vector<double> &q) {
        return !doubledPadding.inCollision(q);
      };
      EXPECT_DOUBLE_EQ(
          zms::manipulation::planCartesianPath(chain, *measured, target,
                                               model.joints(), oldValidity)
              .fraction,
          0.0);
    }
    const auto path = zms::manipulation::planCartesianPath(
        chain, *measured, target, model.joints(), valid);
    EXPECT_DOUBLE_EQ(path.fraction, 1.0) << index;
    if (path.fraction != 1.0) {
      continue;
    }
    for (std::size_t step = 1; step < path.path.size(); ++step) {
      EXPECT_TRUE(valid(path.path[step]));
      double jump = 0.0;
      for (std::size_t joint = 0; joint < measured->size(); ++joint) {
        jump = std::max(jump, std::abs(path.path[step][joint] -
                                       path.path[step - 1][joint]));
      }
      EXPECT_LE(jump, 0.2);
    }
    EXPECT_LT(
        (chain.forward(path.path.back()).translation() - target.translation())
            .norm(),
        0.003);
  }
}
