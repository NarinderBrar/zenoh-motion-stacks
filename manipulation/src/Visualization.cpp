#include <zms/manipulation/Visualization.hpp>

#include <algorithm>
#include <cstddef>
#include <utility>

namespace zms::manipulation {

nlohmann::json trajectoryToJson(const std::string &commandId,
                                const std::vector<std::string> &names,
                                const Trajectory &trajectory, bool pose,
                                const KinematicChain *chain,
                                std::size_t maximumPoints) {
  nlohmann::json points = nlohmann::json::array();
  nlohmann::json eefPath = nlohmann::json::array();
  const std::size_t total = trajectory.points.size();
  if (total > 0) {
    const std::size_t stride =
        std::max<std::size_t>(1, (total + maximumPoints - 1) / maximumPoints);
    for (std::size_t i = 0; i < total; i += stride) {
      nlohmann::json position = nlohmann::json::array();
      for (const double joint : trajectory.points[i]) {
        position.push_back(joint);
      }
      const double time =
          i < trajectory.times.size() ? trajectory.times[i] : 0.0;
      points.push_back({{"t", time}, {"position", std::move(position)}});
      if (pose && chain != nullptr) {
        const auto transform = chain->forward(trajectory.points[i]);
        eefPath.push_back({{"x", transform.translation().x()},
                           {"y", transform.translation().y()},
                           {"z", transform.translation().z()}});
      }
    }
  }
  nlohmann::json payload = {{"contract", 1},
                            {"command_id", commandId},
                            {"name", names},
                            {"points", std::move(points)}};
  if (pose) {
    payload["eef_path"] = std::move(eefPath);
  }
  return payload;
}

}
