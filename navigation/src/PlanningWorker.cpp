#include <zms/navigation/PlanningWorker.hpp>

#include <stdexcept>
#include <utility>

namespace zms::navigation {

PlanningWorker::PlanningWorker()
    : worker_([this](std::stop_token stop) { run(stop); }) {}

PlanningWorker::~PlanningWorker() {
  cancel();
  worker_.request_stop();
  ready_.notify_all();
}

std::uint64_t PlanningWorker::submit(PlanRequest request) {
  if (!request.grid || request.waypoints.empty()) {
    throw std::invalid_argument("plan requires a map and waypoints");
  }
  std::lock_guard lock(mutex_);
  activeStop_.request_stop();
  activeStop_ = std::stop_source{};
  ++generation_;
  pending_ = Job{generation_, std::move(request), activeStop_.get_token()};
  ready_.notify_one();
  return generation_;
}

void PlanningWorker::cancel() {
  std::lock_guard lock(mutex_);
  activeStop_.request_stop();
  pending_.reset();
  ++generation_;
}

std::optional<PlanOutcome> PlanningWorker::poll() {
  std::lock_guard lock(mutex_);
  if (outcomes_.empty()) {
    return std::nullopt;
  }
  auto outcome = std::move(outcomes_.front());
  outcomes_.pop_front();
  return outcome;
}

void PlanningWorker::run(std::stop_token stop) {
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
      Pose current = job->request.start;
      for (const auto &waypoint : job->request.waypoints) {
        auto segment = planGrid(*job->request.grid, current, waypoint,
                                job->request.inflationRadius, job->cancelToken);
        if (outcome.route.size() + segment.size() > 20000) {
          throw std::runtime_error("planned route exceeds 20000 waypoints");
        }
        outcome.route.insert(outcome.route.end(), segment.begin(),
                             segment.end());
        current = {waypoint.x, waypoint.y, waypoint.yaw};
      }
    } catch (const std::exception &error) {
      outcome.error = error.what();
    }
    std::lock_guard lock(mutex_);
    if (job->generation == generation_ && !job->cancelToken.stop_requested()) {
      if (outcomes_.size() == 4) {
        outcomes_.pop_front();
      }
      outcomes_.push_back(std::move(outcome));
    }
  }
}

}
