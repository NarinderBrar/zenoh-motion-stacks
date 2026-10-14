#pragma once

#include <zms/contract/Contract.hpp>

#include <optional>
#include <string>
#include <vector>

namespace zms::manipulation {

struct JointLimit {
  std::string name;
  double lower = 0.0;
  double upper = 0.0;
  double velocity = 0.0;
};

class JointModel {
public:
  static JointModel fromUrdf(const std::string &path,
                             const std::string &baseFrame,
                             const std::string &endEffectorFrame);
  static std::vector<JointLimit>
  limitsFor(const std::string &path, const std::vector<std::string> &names);
  const std::vector<JointLimit> &joints() const;
  std::vector<double>
  orderedPositions(const contract::JointTarget &target) const;
  std::optional<std::vector<double>>
  measuredPositions(const std::vector<std::string> &names,
                    const std::vector<double> &positions) const;

private:
  std::vector<JointLimit> joints_;
};

}
