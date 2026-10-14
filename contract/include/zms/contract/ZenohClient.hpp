#pragma once

#include <nlohmann/json.hpp>
#include <zenoh.hxx>

#include <chrono>
#include <functional>
#include <optional>
#include <string>

namespace zms::contract {

nlohmann::json
query(zenoh::Session &session, const std::string &key,
      const std::optional<nlohmann::json> &payload,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));
void publish(zenoh::Session &session, const std::string &key,
             const nlohmann::json &payload);
nlohmann::json parseJson(const zenoh::Bytes &bytes,
                         std::size_t maximumBytes = 65536);
void reply(zenoh::Query &query, const std::string &key,
           const nlohmann::json &payload);
using QueryHandler =
    std::function<nlohmann::json(const std::optional<nlohmann::json> &)>;
zenoh::Queryable<void> declareJsonQueryable(zenoh::Session &session,
                                            const std::string &key,
                                            QueryHandler handler,
                                            std::size_t maximumBytes = 65536);

}
