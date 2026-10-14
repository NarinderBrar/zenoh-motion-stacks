#include <zms/manipulation/Planner.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

namespace zms::manipulation {
namespace {

double distance(const std::vector<double> &a, const std::vector<double> &b) {
  double total = 0.0;
  for (std::size_t index = 0; index < a.size(); ++index) {
    total += (a[index] - b[index]) * (a[index] - b[index]);
  }
  return std::sqrt(total);
}

std::vector<double> interpolate(const std::vector<double> &a,
                                const std::vector<double> &b, double ratio) {
  std::vector<double> result(a.size());
  for (std::size_t index = 0; index < a.size(); ++index) {
    result[index] = a[index] + (b[index] - a[index]) * ratio;
  }
  return result;
}

struct Tree {
  JointPath states;
  std::vector<int> parents;

  int nearest(const std::vector<double> &target) const {
    int best = 0;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < states.size(); ++index) {
      const double candidate = distance(states[index], target);
      if (candidate < bestDistance) {
        bestDistance = candidate;
        best = static_cast<int>(index);
      }
    }
    return best;
  }

  JointPath pathTo(int index) const {
    JointPath result;
    for (int current = index; current >= 0; current = parents[current]) {
      result.push_back(states[current]);
    }
    std::reverse(result.begin(), result.end());
    return result;
  }
};

enum class Extension { Trapped, Advanced, Reached };

Extension extend(Tree &tree, const std::vector<double> &target,
                 const StateValidity &valid, const PlannerOptions &options) {
  const int nearest = tree.nearest(target);
  const auto &from = tree.states[nearest];
  const double gap = distance(from, target);
  const bool reaches = gap <= options.stepSize;
  const auto next =
      reaches ? target : interpolate(from, target, options.stepSize / gap);
  if (!motionValid(from, next, valid, options.checkResolution)) {
    return Extension::Trapped;
  }
  tree.states.push_back(next);
  tree.parents.push_back(nearest);
  return reaches ? Extension::Reached : Extension::Advanced;
}

void shortcut(JointPath &path, const StateValidity &valid,
              const PlannerOptions &options, std::mt19937 &random) {
  for (int iteration = 0;
       iteration < options.shortcutIterations && path.size() > 2; ++iteration) {
    std::uniform_int_distribution<std::size_t> pick(0, path.size() - 1);
    std::size_t first = pick(random);
    std::size_t second = pick(random);
    if (first > second) {
      std::swap(first, second);
    }
    if (second <= first + 1) {
      continue;
    }
    if (motionValid(path[first], path[second], valid,
                    options.checkResolution)) {
      path.erase(path.begin() + static_cast<std::ptrdiff_t>(first) + 1,
                 path.begin() + static_cast<std::ptrdiff_t>(second));
    }
  }
}

double trapezoidTime(double distanceToCover, double maxVelocity,
                     double maxAcceleration) {
  if (distanceToCover <= 0.0) {
    return 0.0;
  }
  if (distanceToCover <= maxVelocity * maxVelocity / maxAcceleration) {
    return 2.0 * std::sqrt(distanceToCover / maxAcceleration);
  }
  return distanceToCover / maxVelocity + maxVelocity / maxAcceleration;
}

double trapezoidPosition(double time, double total, double maxVelocity,
                         double maxAcceleration) {
  const double accelerationTime =
      std::min(maxVelocity / maxAcceleration, total / 2.0);
  const double peak = maxAcceleration * accelerationTime;
  if (time <= accelerationTime) {
    return 0.5 * maxAcceleration * time * time;
  }
  const double accelerationDistance =
      0.5 * maxAcceleration * accelerationTime * accelerationTime;
  if (time <= total - accelerationTime) {
    return accelerationDistance + peak * (time - accelerationTime);
  }
  const double remaining = total - time;
  const double fullDistance =
      2.0 * accelerationDistance + peak * (total - 2.0 * accelerationTime);
  return fullDistance - 0.5 * maxAcceleration * remaining * remaining;
}

}

bool motionValid(const std::vector<double> &from, const std::vector<double> &to,
                 const StateValidity &valid, double resolution) {
  if (resolution <= 0.0) {
    throw std::invalid_argument("check resolution must be positive");
  }
  const int steps =
      std::max(1, static_cast<int>(std::ceil(distance(from, to) / resolution)));
  for (int step = 1; step <= steps; ++step) {
    if (!valid(interpolate(from, to, static_cast<double>(step) / steps))) {
      return false;
    }
  }
  return true;
}

std::optional<JointPath>
planJointPath(const std::vector<double> &start, const std::vector<double> &goal,
              const std::vector<JointLimit> &limits, const StateValidity &valid,
              const PlannerOptions &options, std::stop_token stop,
              std::string *error) {
  const auto fail = [error](std::string message) -> std::optional<JointPath> {
    if (error != nullptr) {
      *error = std::move(message);
    }
    return std::nullopt;
  };
  if (start.size() != limits.size() || goal.size() != limits.size() ||
      options.stepSize <= 0.0 || options.maxSeconds <= 0.0) {
    return fail("invalid planning request");
  }
  if (!valid(start)) {
    return fail("start_in_collision");
  }
  if (!valid(goal)) {
    return fail("goal_in_collision");
  }
  if (motionValid(start, goal, valid, options.checkResolution)) {
    return JointPath{start, goal};
  }
  std::mt19937 random(options.seed);
  Tree fromStart{{start}, {-1}};
  Tree fromGoal{{goal}, {-1}};
  Tree *grow = &fromStart;
  Tree *other = &fromGoal;
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(options.maxSeconds));
  std::vector<std::uniform_real_distribution<double>> ranges;
  for (const auto &limit : limits) {
    ranges.emplace_back(limit.lower, limit.upper);
  }
  while (std::chrono::steady_clock::now() < deadline) {
    if (stop.stop_requested()) {
      return fail("cancelled");
    }
    std::vector<double> sample(limits.size());
    for (std::size_t index = 0; index < limits.size(); ++index) {
      sample[index] = ranges[index](random);
    }
    if (extend(*grow, sample, valid, options) != Extension::Trapped) {
      const auto newest = grow->states.back();
      Extension connection = Extension::Advanced;
      while (connection == Extension::Advanced) {
        connection = extend(*other, newest, valid, options);
      }
      if (connection == Extension::Reached) {
        auto first =
            fromStart.pathTo(static_cast<int>(fromStart.states.size() - 1));
        auto second =
            fromGoal.pathTo(static_cast<int>(fromGoal.states.size() - 1));
        std::reverse(second.begin(), second.end());
        first.insert(first.end(), second.begin() + 1, second.end());
        shortcut(first, valid, options, random);
        return first;
      }
    }
    std::swap(grow, other);
  }
  return fail("planning_timeout");
}

PlanningThread::PlanningThread(StateValidity valid,
                               std::vector<JointLimit> limits,
                               PlannerOptions options)
    : valid_(std::move(valid)), limits_(std::move(limits)), options_(options),
      worker_([this](std::stop_token stop) { run(stop); }) {}

PlanningThread::~PlanningThread() {
  cancel();
  worker_.request_stop();
  ready_.notify_all();
}

std::uint64_t PlanningThread::submit(std::vector<double> start,
                                     std::vector<double> goal) {
  std::lock_guard lock(mutex_);
  activeStop_.request_stop();
  activeStop_ = std::stop_source{};
  ++generation_;
  pending_ = Job{generation_, std::move(start), std::move(goal),
                 activeStop_.get_token()};
  outcome_.reset();
  ready_.notify_one();
  return generation_;
}

void PlanningThread::cancel() {
  std::lock_guard lock(mutex_);
  activeStop_.request_stop();
  pending_.reset();
  outcome_.reset();
  ++generation_;
}

std::optional<PlanOutcome> PlanningThread::poll() {
  std::lock_guard lock(mutex_);
  auto outcome = std::move(outcome_);
  outcome_.reset();
  return outcome;
}

void PlanningThread::run(std::stop_token stop) {
  while (!stop.stop_requested()) {
    std::optional<Job> job;
    {
      std::unique_lock lock(mutex_);
      ready_.wait(lock, stop, [&] { return pending_.has_value(); });
      if (stop.stop_requested()) {
        return;
      }
      job = std::move(pending_);
      pending_.reset();
    }
    PlanOutcome outcome;
    outcome.generation = job->generation;
    try {
      outcome.path = planJointPath(job->start, job->goal, limits_, valid_,
                                   options_, job->cancelToken, &outcome.error);
    } catch (const std::exception &error) {
      outcome.error = error.what();
    }
    std::lock_guard lock(mutex_);
    if (job->generation == generation_ && !job->cancelToken.stop_requested()) {
      outcome_ = std::move(outcome);
    }
  }
}

double Trajectory::duration() const {
  return times.empty() ? 0.0 : times.back();
}

std::vector<double> Trajectory::sample(double time) const {
  if (points.empty()) {
    throw std::invalid_argument("empty trajectory");
  }
  if (time <= 0.0 || points.size() == 1) {
    return points.front();
  }
  if (time >= duration()) {
    return points.back();
  }
  if (continuous) {
    const double reached = trapezoidPosition(time, duration(), peakProgressRate,
                                             peakProgressAcceleration);
    const double target = std::clamp(reached, 0.0, progress.back());
    const auto upper =
        std::upper_bound(progress.begin(), progress.end(), target);
    const std::size_t segment = std::min<std::size_t>(
        static_cast<std::size_t>(std::distance(progress.begin(), upper)) - 1,
        points.size() - 2);
    const double span = progress[segment + 1] - progress[segment];
    const double ratio =
        span > 0.0 ? std::clamp((target - progress[segment]) / span, 0.0, 1.0)
                   : 1.0;
    return interpolate(points[segment], points[segment + 1], ratio);
  }
  const auto upper = std::upper_bound(times.begin(), times.end(), time);
  const std::size_t segment =
      static_cast<std::size_t>(std::distance(times.begin(), upper)) - 1;
  const double segmentTime = times[segment + 1] - times[segment];
  const double local = time - times[segment];
  const double ratio =
      std::clamp(trapezoidPosition(local, segmentTime, peakVelocity[segment],
                                   peakAcceleration[segment]),
                 0.0, 1.0);
  return interpolate(points[segment], points[segment + 1], ratio);
}

Trajectory timeParameterize(const JointPath &path,
                            const std::vector<JointLimit> &limits,
                            double maxAcceleration) {
  if (path.empty() || maxAcceleration <= 0.0) {
    throw std::invalid_argument("trajectory needs a path and acceleration");
  }
  Trajectory trajectory;
  trajectory.points = path;
  trajectory.times.push_back(0.0);
  for (std::size_t segment = 0; segment + 1 < path.size(); ++segment) {
    double unitVelocity = std::numeric_limits<double>::infinity();
    double unitAcceleration = std::numeric_limits<double>::infinity();
    for (std::size_t joint = 0; joint < limits.size(); ++joint) {
      const double span =
          std::abs(path[segment + 1][joint] - path[segment][joint]);
      if (span <= 1e-12) {
        continue;
      }
      unitVelocity = std::min(unitVelocity, limits[joint].velocity / span);
      unitAcceleration = std::min(unitAcceleration, maxAcceleration / span);
    }
    if (!std::isfinite(unitVelocity)) {
      unitVelocity = 1.0;
      unitAcceleration = 1.0;
    }
    const double segmentTime =
        trapezoidTime(1.0, unitVelocity, unitAcceleration);
    trajectory.peakVelocity.push_back(unitVelocity);
    trajectory.peakAcceleration.push_back(unitAcceleration);
    trajectory.times.push_back(trajectory.times.back() +
                               std::max(segmentTime, 1e-6));
  }
  return trajectory;
}

Trajectory timeParameterizeContinuous(const JointPath &path,
                                      const std::vector<JointLimit> &limits,
                                      double maxAcceleration) {
  if (path.size() < 2 || maxAcceleration <= 0.0) {
    return timeParameterize(path, limits, maxAcceleration);
  }
  Trajectory trajectory;
  trajectory.continuous = true;
  trajectory.points = path;
  trajectory.progress.push_back(0.0);
  double fastestJoint = 0.0;
  for (const auto &limit : limits) {
    fastestJoint = std::max(fastestJoint, limit.velocity);
  }
  for (std::size_t segment = 0; segment + 1 < path.size(); ++segment) {
    double slowest = 0.0;
    for (std::size_t joint = 0; joint < limits.size(); ++joint) {
      slowest = std::max(
          slowest, std::abs(path[segment + 1][joint] - path[segment][joint]) /
                       limits[joint].velocity);
    }
    trajectory.progress.push_back(trajectory.progress.back() + slowest);
  }
  const double total = trajectory.progress.back();
  trajectory.peakProgressRate = 1.0;
  trajectory.peakProgressAcceleration = maxAcceleration / fastestJoint;
  const double duration =
      total > 0.0 ? trapezoidTime(total, trajectory.peakProgressRate,
                                  trajectory.peakProgressAcceleration)
                  : 0.0;
  trajectory.times = {0.0, std::max(duration, 1e-6)};
  return trajectory;
}

CartesianPath planCartesianPath(const KinematicChain &chain,
                                const std::vector<double> &start,
                                const Eigen::Isometry3d &target,
                                const std::vector<JointLimit> &limits,
                                const StateValidity &valid, double positionStep,
                                double angleStep, double maxJointJump) {
  if (positionStep <= 0.0 || angleStep <= 0.0 || maxJointJump <= 0.0) {
    throw std::invalid_argument("invalid cartesian step sizes");
  }
  const Eigen::Isometry3d origin = chain.forward(start);
  const Eigen::Quaterniond from(origin.linear());
  const Eigen::Quaterniond to(target.linear());
  const double distanceToGo =
      (target.translation() - origin.translation()).norm();
  const double angleToGo = from.angularDistance(to);
  const int steps =
      std::max(1, static_cast<int>(std::ceil(std::max(
                      distanceToGo / positionStep, angleToGo / angleStep))));
  CartesianPath result;
  result.path.push_back(start);
  IkOptions options;
  options.attempts = 1;
  options.maxSolutions = 1;
  std::vector<double> previous = start;
  for (int step = 1; step <= steps; ++step) {
    const double ratio = static_cast<double>(step) / steps;
    Eigen::Isometry3d waypoint = Eigen::Isometry3d::Identity();
    waypoint.translation() =
        origin.translation() +
        ratio * (target.translation() - origin.translation());
    waypoint.linear() = from.slerp(ratio, to).toRotationMatrix();
    const auto solution = chain.solveIk(waypoint, previous, limits, options);
    if (!solution) {
      break;
    }
    double jump = 0.0;
    for (std::size_t joint = 0; joint < previous.size(); ++joint) {
      jump = std::max(jump, std::abs((*solution)[joint] - previous[joint]));
    }
    if (jump > maxJointJump || !valid(*solution)) {
      break;
    }
    result.path.push_back(*solution);
    previous = *solution;
    result.fraction = ratio;
  }
  return result;
}

}
