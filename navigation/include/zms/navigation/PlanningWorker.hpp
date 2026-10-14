#pragma once

#include <zms/navigation/GridPlanner.hpp>

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <thread>

namespace zms::navigation {

struct PlanRequest {
  std::shared_ptr<const OccupancyGrid> grid;
  Pose start;
  std::vector<contract::Waypoint> waypoints;
  double inflationRadius = 0.0;
};

struct PlanOutcome {
  std::uint64_t generation = 0;
  std::vector<contract::Waypoint> route;
  std::string error;
};

class PlanningWorker {
public:
  PlanningWorker();
  PlanningWorker(const PlanningWorker &) = delete;
  PlanningWorker &operator=(const PlanningWorker &) = delete;
  ~PlanningWorker();

  std::uint64_t submit(PlanRequest request);
  void cancel();
  std::optional<PlanOutcome> poll();

private:
  struct Job {
    std::uint64_t generation;
    PlanRequest request;
    std::stop_token cancelToken;
  };

  void run(std::stop_token stop);

  std::mutex mutex_;
  std::condition_variable_any ready_;
  std::optional<Job> pending_;
  std::deque<PlanOutcome> outcomes_;
  std::stop_source activeStop_;
  std::uint64_t generation_ = 0;
  std::jthread worker_;
};

}
