#include <zms/navigation/Localization.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace zms::navigation {
namespace {

int cellIndex(const OccupancyGrid &grid, double x, double y) {
  const double dx = x - grid.origin.x;
  const double dy = y - grid.origin.y;
  const double cosine = std::cos(grid.origin.yaw);
  const double sine = std::sin(grid.origin.yaw);
  const int cellX =
      static_cast<int>(std::floor((cosine * dx + sine * dy) / grid.resolution));
  const int cellY = static_cast<int>(
      std::floor((-sine * dx + cosine * dy) / grid.resolution));
  if (cellX < 0 || cellY < 0 || cellX >= grid.width || cellY >= grid.height) {
    return -1;
  }
  return cellY * grid.width + cellX;
}

}

Pose compose(const Pose &a, const Pose &b) {
  const double cosine = std::cos(a.yaw);
  const double sine = std::sin(a.yaw);
  return {a.x + cosine * b.x - sine * b.y, a.y + sine * b.x + cosine * b.y,
          normalizeAngle(a.yaw + b.yaw)};
}

Pose inverse(const Pose &pose) {
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return {-cosine * pose.x - sine * pose.y, sine * pose.x - cosine * pose.y,
          normalizeAngle(-pose.yaw)};
}

LikelihoodField::LikelihoodField(const OccupancyGrid &grid, double maxDistance)
    : grid_(grid), maxDistance_(maxDistance),
      distances_(grid.cells.size(), maxDistance) {
  std::deque<int> frontier;
  std::vector<int> source(grid.cells.size(), -1);
  for (int index = 0; index < static_cast<int>(grid.cells.size()); ++index) {
    if (grid.cells[index] >= 50) {
      distances_[index] = 0.0;
      source[index] = index;
      frontier.push_back(index);
    }
  }
  constexpr int offsets[8][2] = {{-1, 0},  {1, 0},  {0, -1}, {0, 1},
                                 {-1, -1}, {1, -1}, {-1, 1}, {1, 1}};
  while (!frontier.empty()) {
    const int current = frontier.front();
    frontier.pop_front();
    const int x = current % grid.width;
    const int y = current / grid.width;
    const int sx = source[current] % grid.width;
    const int sy = source[current] / grid.width;
    for (const auto &offset : offsets) {
      const int nx = x + offset[0];
      const int ny = y + offset[1];
      if (nx < 0 || ny < 0 || nx >= grid.width || ny >= grid.height) {
        continue;
      }
      const int next = ny * grid.width + nx;
      const double distance = std::hypot(nx - sx, ny - sy) * grid.resolution;
      if (distance >= distances_[next] - 1e-9 || distance > maxDistance) {
        continue;
      }
      distances_[next] = distance;
      source[next] = source[current];
      frontier.push_back(next);
    }
  }
}

double LikelihoodField::distance(double x, double y) const {
  const int index = cellIndex(grid_, x, y);
  return index < 0 ? maxDistance_ : distances_[index];
}

ParticleFilter::ParticleFilter(LocalizationConfig config)
    : config_(config), random_(config.seed) {
  if (config_.particles < 10 || config_.beams < 4 || config_.hitSigma <= 0.0) {
    throw std::invalid_argument("invalid localization configuration");
  }
}

void ParticleFilter::setMap(std::shared_ptr<const OccupancyGrid> map) {
  map_ = std::move(map);
  field_ = map_ ? std::make_unique<LikelihoodField>(*map_, 2.0) : nullptr;
}

bool ParticleFilter::hasMap() const { return field_ != nullptr; }

void ParticleFilter::initialize(const Pose &pose) {
  std::normal_distribution<double> position(0.0, config_.initialSpread);
  std::normal_distribution<double> heading(0.0, config_.initialYawSpread);
  particles_.assign(static_cast<std::size_t>(config_.particles), {});
  for (std::size_t index = 0; index < particles_.size(); ++index) {
    particles_[index].pose =
        index == 0
            ? pose
            : Pose{pose.x + position(random_), pose.y + position(random_),
                   normalizeAngle(pose.yaw + heading(random_))};
    particles_[index].weight = 1.0 / config_.particles;
  }
  initialized_ = true;
  refreshEstimate();
  if (lastOdom_) {
    odomToMap_ = compose(estimate_, inverse(*lastOdom_));
  }
}

bool ParticleFilter::initialized() const { return initialized_; }

bool ParticleFilter::update(const Pose &odom, const std::vector<Point2> &scan) {
  if (!lastOdom_) {
    lastOdom_ = odom;
    if (initialized_) {
      odomToMap_ = compose(estimate_, inverse(odom));
    }
    return false;
  }
  if (!initialized_ || !field_) {
    lastOdom_ = odom;
    return false;
  }
  const Pose delta = compose(inverse(*lastOdom_), odom);
  if (std::hypot(delta.x, delta.y) < config_.updateDistance &&
      std::abs(delta.yaw) < config_.updateAngle) {
    odomToMap_ = compose(estimate_, inverse(*lastOdom_));
    return false;
  }
  predict(*lastOdom_, odom);
  lastOdom_ = odom;
  if (!scan.empty()) {
    weigh(scan);
    resample();
  }
  refreshEstimate();
  odomToMap_ = compose(estimate_, inverse(odom));
  return true;
}

void ParticleFilter::predict(const Pose &from, const Pose &to) {
  const Pose delta = compose(inverse(from), to);
  const double translation = std::hypot(delta.x, delta.y);
  const double firstRotation =
      translation < 1e-6 ? 0.0 : std::atan2(delta.y, delta.x);
  const double secondRotation = normalizeAngle(delta.yaw - firstRotation);
  const double r1 = firstRotation * firstRotation;
  const double r2 = secondRotation * secondRotation;
  const double tt = translation * translation;
  std::normal_distribution<double> rotation1(
      0.0, std::sqrt(config_.rotationFromRotation * r1 +
                     config_.rotationFromTranslation * tt) +
               1e-6);
  std::normal_distribution<double> move(
      0.0, std::sqrt(config_.translationFromTranslation * tt +
                     config_.translationFromRotation * (r1 + r2)) +
               1e-6);
  std::normal_distribution<double> rotation2(
      0.0, std::sqrt(config_.rotationFromRotation * r2 +
                     config_.rotationFromTranslation * tt) +
               1e-6);
  for (auto &particle : particles_) {
    const double turn1 = firstRotation + rotation1(random_);
    const double distance = translation + move(random_);
    const double turn2 = secondRotation + rotation2(random_);
    particle.pose = compose(particle.pose, {distance * std::cos(turn1),
                                            distance * std::sin(turn1),
                                            normalizeAngle(turn1 + turn2)});
  }
}

void ParticleFilter::weigh(const std::vector<Point2> &scan) {
  const std::size_t stride = std::max<std::size_t>(
      1, scan.size() / static_cast<std::size_t>(config_.beams));
  const double hitScale = 1.0 / (2.0 * config_.hitSigma * config_.hitSigma);
  const double randomTerm = config_.randomWeight / config_.maxRange;
  double maximum = -std::numeric_limits<double>::infinity();
  std::vector<double> logWeights(particles_.size(), 0.0);
  for (std::size_t p = 0; p < particles_.size(); ++p) {
    const auto &pose = particles_[p].pose;
    const double cosine = std::cos(pose.yaw);
    const double sine = std::sin(pose.yaw);
    double logWeight = 0.0;
    for (std::size_t beam = 0; beam < scan.size(); beam += stride) {
      const auto &point = scan[beam];
      const double x = pose.x + cosine * point.x - sine * point.y;
      const double y = pose.y + sine * point.x + cosine * point.y;
      const double distance = field_->distance(x, y);
      logWeight += std::log((1.0 - config_.randomWeight) *
                                std::exp(-distance * distance * hitScale) +
                            randomTerm);
    }
    logWeights[p] = logWeight;
    maximum = std::max(maximum, logWeight);
  }
  double total = 0.0;
  for (std::size_t p = 0; p < particles_.size(); ++p) {
    particles_[p].weight *= std::exp(logWeights[p] - maximum);
    total += particles_[p].weight;
  }
  for (auto &particle : particles_) {
    particle.weight = total > 0.0
                          ? particle.weight / total
                          : 1.0 / static_cast<double>(particles_.size());
  }
}

void ParticleFilter::resample() {
  double squared = 0.0;
  for (const auto &particle : particles_) {
    squared += particle.weight * particle.weight;
  }
  if (squared <= 0.0 ||
      1.0 / squared > 0.5 * static_cast<double>(particles_.size())) {
    return;
  }
  std::vector<Particle> next;
  next.reserve(particles_.size());
  const double step = 1.0 / static_cast<double>(particles_.size());
  std::uniform_real_distribution<double> start(0.0, step);
  double target = start(random_);
  double cumulative = particles_[0].weight;
  std::size_t index = 0;
  for (std::size_t count = 0; count < particles_.size(); ++count) {
    while (target > cumulative && index + 1 < particles_.size()) {
      ++index;
      cumulative += particles_[index].weight;
    }
    next.push_back({particles_[index].pose, step});
    target += step;
  }
  particles_ = std::move(next);
}

void ParticleFilter::refreshEstimate() {
  double x = 0.0;
  double y = 0.0;
  double sine = 0.0;
  double cosine = 0.0;
  double total = 0.0;
  for (const auto &particle : particles_) {
    x += particle.weight * particle.pose.x;
    y += particle.weight * particle.pose.y;
    sine += particle.weight * std::sin(particle.pose.yaw);
    cosine += particle.weight * std::cos(particle.pose.yaw);
    total += particle.weight;
  }
  if (total > 0.0) {
    estimate_ = {x / total, y / total, std::atan2(sine, cosine)};
  }
}

Pose ParticleFilter::estimate() const { return estimate_; }

Pose ParticleFilter::odomToMap() const { return odomToMap_; }

Pose ParticleFilter::toMap(const Pose &odom) const {
  return compose(odomToMap_, odom);
}

double ParticleFilter::positionSpread() const {
  double variance = 0.0;
  for (const auto &particle : particles_) {
    variance += particle.weight * (std::pow(particle.pose.x - estimate_.x, 2) +
                                   std::pow(particle.pose.y - estimate_.y, 2));
  }
  return std::sqrt(variance);
}

const std::vector<Particle> &ParticleFilter::particles() const {
  return particles_;
}

}
