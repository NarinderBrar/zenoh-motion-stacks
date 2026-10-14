#include <zms/manipulation/JointMotion.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace zms::manipulation {

JointStep stepTowards(double position, double velocity, double goal,
                      double maxVelocity, double maxAcceleration,
                      double elapsedSeconds) {
  if (!std::isfinite(position) || !std::isfinite(velocity) ||
      !std::isfinite(goal) || !std::isfinite(maxVelocity) ||
      maxVelocity <= 0.0 || !std::isfinite(maxAcceleration) ||
      maxAcceleration <= 0.0 || !std::isfinite(elapsedSeconds) ||
      elapsedSeconds <= 0.0) {
    throw std::invalid_argument("invalid joint motion step");
  }
  const double difference = goal - position;
  if (difference == 0.0) {
    return {goal, 0.0};
  }
  const double distance = std::abs(difference);
  const double velocityChange = maxAcceleration * elapsedSeconds;
  const double halfChange = velocityChange / 2.0;
  const double brakingSpeed =
      std::sqrt(halfChange * halfChange + 2.0 * maxAcceleration * distance) -
      halfChange;
  const double desired =
      std::copysign(std::min(maxVelocity, brakingSpeed), difference);
  const double next = velocity + std::clamp(desired - velocity, -velocityChange,
                                            velocityChange);
  const double step = next * elapsedSeconds;
  if (step * difference > 0.0 && std::abs(step) >= distance) {
    return {goal, next};
  }
  return {position + step, next};
}

}
