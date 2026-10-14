#include <zms/contract/Contract.hpp>
#include <zms/contract/ZenohClient.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace {

using Json = nlohmann::json;

class Endpoint {
public:
  virtual ~Endpoint() = default;
  virtual Json request(std::string_view suffix,
                       const std::optional<Json> &payload) = 0;
};

class FakeNavigationEndpoint final : public Endpoint {
public:
  Json request(std::string_view suffix,
               const std::optional<Json> &payload) override {
    if (suffix == "navigation/status") {
      if (!active_) {
        return {{"contract", 1},
                {"state", "idle"},
                {"stack_version", "fake"},
                {"features", Json::array({"waypoint_route"})}};
      }
      auto response =
          zms::contract::status(active_->commandId, active_->clientId,
                                zms::contract::CommandState::Active);
      response["stack_version"] = "fake";
      response["features"] = Json::array({"waypoint_route"});
      return response;
    }
    if (!payload) {
      return zms::contract::rejected("", "", "invalid_request",
                                     "request payload is required");
    }
    try {
      if (suffix == "navigation/goal") {
        const auto goal = zms::contract::parseGoal(*payload);
        if (active_ && active_->clientId != goal.clientId) {
          return zms::contract::rejected(goal.commandId, goal.clientId, "busy",
                                         "another client owns the command");
        }
        active_ = goal;
        return zms::contract::accepted(goal.commandId, goal.clientId);
      }
      if (suffix == "navigation/cancel") {
        const auto cancel = zms::contract::parseCancel(*payload);
        if (!active_) {
          return {{"contract", 1}, {"state", "idle"}};
        }
        if (active_->clientId != cancel.clientId ||
            active_->commandId != cancel.commandId) {
          return zms::contract::rejected(cancel.commandId, cancel.clientId,
                                         "not_owner",
                                         "command belongs to another client");
        }
        active_.reset();
        return zms::contract::accepted(cancel.commandId, cancel.clientId);
      }
    } catch (const std::exception &error) {
      return zms::contract::rejected("", "", "invalid_request", error.what());
    }
    return zms::contract::rejected("", "", "unknown_key", "unknown key");
  }

private:
  std::optional<zms::contract::Goal> active_;
};

class ZenohNavigationEndpoint final : public Endpoint {
public:
  explicit ZenohNavigationEndpoint(std::string prefix)
      : session_(zenoh::Config::create_default()), prefix_(std::move(prefix)) {}

  Json request(std::string_view suffix,
               const std::optional<Json> &payload) override {
    return zms::contract::query(session_, zms::contract::key(prefix_, suffix),
                                payload);
  }

private:
  zenoh::Session session_;
  std::string prefix_;
};

std::unique_ptr<Endpoint> endpoint() {
  const char *prefix = std::getenv("ZMS_CONFORMANCE_PREFIX");
  if (prefix && *prefix) {
    return std::make_unique<ZenohNavigationEndpoint>(prefix);
  }
  return std::make_unique<FakeNavigationEndpoint>();
}

}

TEST(Conformance, NavigationOwnershipAndVersion) {
  auto stack = endpoint();
  const auto initial = stack->request("navigation/status", std::nullopt);
  EXPECT_EQ(initial.at("contract"), 1);
  EXPECT_TRUE(initial.at("features").is_array());

  const auto first = zms::contract::toJson(zms::contract::Goal{
      "conformance-1", "conformance-owner", {{100.0, 0.0, 0.0}}});
  const auto accepted = stack->request("navigation/goal", first);
  ASSERT_TRUE(accepted.at("accepted").get<bool>());
  EXPECT_EQ(accepted.at("command_id"), "conformance-1");

  const auto busy = stack->request(
      "navigation/goal",
      zms::contract::toJson(zms::contract::Goal{
          "conformance-2", "another-client", {{101.0, 0.0, 0.0}}}));
  ASSERT_FALSE(busy.at("accepted").get<bool>());
  EXPECT_EQ(busy.at("reason"), "busy");

  const auto notOwner = stack->request(
      "navigation/cancel", zms::contract::toJson(zms::contract::Cancel{
                               "conformance-1", "another-client"}));
  ASSERT_FALSE(notOwner.at("accepted").get<bool>());
  EXPECT_EQ(notOwner.at("reason"), "not_owner");

  auto wrongVersion = first;
  wrongVersion["contract"] = 2;
  const auto invalid = stack->request("navigation/goal", wrongVersion);
  ASSERT_FALSE(invalid.at("accepted").get<bool>());
  EXPECT_EQ(invalid.at("reason"), "invalid_request");

  const auto cancelled = stack->request(
      "navigation/cancel", zms::contract::toJson(zms::contract::Cancel{
                               "conformance-1", "conformance-owner"}));
  EXPECT_TRUE(cancelled.at("accepted").get<bool>());
  const auto idle = stack->request("navigation/cancel",
                                   zms::contract::toJson(zms::contract::Cancel{
                                       "conformance-1", "conformance-owner"}));
  EXPECT_EQ(idle.at("state"), "idle");
}
