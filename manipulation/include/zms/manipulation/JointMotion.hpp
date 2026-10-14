#pragma once

namespace zms::manipulation {

struct JointStep {
  double position = 0.0;
  double velocity = 0.0;
};

JointStep stepTowards(double position, double velocity, double goal,
                      double maxVelocity, double maxAcceleration,
                      double elapsedSeconds);

}
