#pragma once

#include <zms/manipulation/Kinematics.hpp>
#include <zms/manipulation/Planner.hpp>

#include <nlohmann/json.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace zms::manipulation {

nlohmann::json trajectoryToJson(const std::string &commandId,
                                const std::vector<std::string> &names,
                                const Trajectory &trajectory, bool pose,
                                const KinematicChain *chain,
                                std::size_t maximumPoints = 200);

}
