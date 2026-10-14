#pragma once

#include <nlohmann/json.hpp>
#include <zenoh.hxx>

#include <chrono>
#include <optional>
#include <string>

namespace zms::contract {

nlohmann::json
query(zenoh::Session &session, const std::string &key,
      const std::optional<nlohmann::json> &payload,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));
void publish(zenoh::Session &session, const std::string &key,
             const nlohmann::json &payload);

}
