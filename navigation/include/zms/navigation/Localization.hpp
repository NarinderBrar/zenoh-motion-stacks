#pragma once

#include <zms/navigation/GridPlanner.hpp>
#include <zms/navigation/LocalPlanner.hpp>
#include <zms/navigation/Navigation.hpp>

#include <memory>
#include <optional>
#include <random>
#include <vector>

namespace zms::navigation {

struct LocalizationConfig {
  int particles = 400;
  int beams = 60;
  double hitSigma = 0.2;
  double randomWeight = 0.05;
  double maxRange = 8.0;
  double updateDistance = 0.05;
  double updateAngle = 0.05;
  double rotationFromRotation = 0.2;
  double rotationFromTranslation = 0.2;
  double translationFromTranslation = 0.2;
  double translationFromRotation = 0.2;
  double initialSpread = 0.3;
  double initialYawSpread = 0.3;
  unsigned seed = 1;
};

struct Particle {
  Pose pose;
  double weight = 1.0;
};

class LikelihoodField {
public:
  LikelihoodField(const OccupancyGrid &grid, double maxDistance);
  double distance(double x, double y) const;

private:
  OccupancyGrid grid_;
  double maxDistance_;
  std::vector<double> distances_;
};

Pose compose(const Pose &a, const Pose &b);
Pose inverse(const Pose &pose);

class ParticleFilter {
public:
  explicit ParticleFilter(LocalizationConfig config);

  void setMap(std::shared_ptr<const OccupancyGrid> map);
  bool hasMap() const;
  void initialize(const Pose &pose);
  bool initialized() const;
  bool update(const Pose &odom, const std::vector<Point2> &scanInBase);
  Pose estimate() const;
  Pose odomToMap() const;
  Pose toMap(const Pose &odom) const;
  double positionSpread() const;
  const std::vector<Particle> &particles() const;

private:
  void predict(const Pose &from, const Pose &to);
  void weigh(const std::vector<Point2> &scan);
  void resample();
  void refreshEstimate();

  LocalizationConfig config_;
  std::shared_ptr<const OccupancyGrid> map_;
  std::unique_ptr<LikelihoodField> field_;
  std::vector<Particle> particles_;
  std::mt19937 random_;
  std::optional<Pose> lastOdom_;
  Pose estimate_;
  Pose odomToMap_;
  bool initialized_ = false;
};

}
