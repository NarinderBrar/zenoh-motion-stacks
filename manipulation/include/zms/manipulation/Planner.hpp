#pragma once

#include <zms/manipulation/JointModel.hpp>

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

namespace zms::manipulation {

using JointPath = std::vector<std::vector<double>>;
using StateValidity = std::function<bool(const std::vector<double> &)>;

struct PlannerOptions {
  double maxSeconds = 2.0;
  double stepSize = 0.2;
  double checkResolution = 0.02;
  int shortcutIterations = 150;
  unsigned seed = 1;
};

bool motionValid(const std::vector<double> &from, const std::vector<double> &to,
                 const StateValidity &valid, double resolution);

std::optional<JointPath>
planJointPath(const std::vector<double> &start, const std::vector<double> &goal,
              const std::vector<JointLimit> &limits, const StateValidity &valid,
              const PlannerOptions &options, std::stop_token stop = {},
              std::string *error = nullptr);

struct Trajectory {
  std::vector<double> times;
  JointPath points;
  std::vector<double> peakVelocity;
  std::vector<double> peakAcceleration;

  double duration() const;
  std::vector<double> sample(double time) const;
};

struct PlanOutcome {
  std::uint64_t generation = 0;
  std::optional<JointPath> path;
  std::string error;
};

class PlanningThread {
public:
  PlanningThread(StateValidity valid, std::vector<JointLimit> limits,
                 PlannerOptions options);
  PlanningThread(const PlanningThread &) = delete;
  PlanningThread &operator=(const PlanningThread &) = delete;
  ~PlanningThread();

  std::uint64_t submit(std::vector<double> start, std::vector<double> goal);
  void cancel();
  std::optional<PlanOutcome> poll();

private:
  struct Job {
    std::uint64_t generation;
    std::vector<double> start;
    std::vector<double> goal;
    std::stop_token cancelToken;
  };

  void run(std::stop_token stop);

  StateValidity valid_;
  std::vector<JointLimit> limits_;
  PlannerOptions options_;
  std::mutex mutex_;
  std::condition_variable_any ready_;
  std::optional<Job> pending_;
  std::optional<PlanOutcome> outcome_;
  std::stop_source activeStop_;
  std::uint64_t generation_ = 0;
  std::jthread worker_;
};

Trajectory timeParameterize(const JointPath &path,
                            const std::vector<JointLimit> &limits,
                            double maxAcceleration);

}
