#include <zms/contract/ZenohClient.hpp>

#include <atomic>
#include <future>
#include <memory>
#include <stdexcept>

namespace zms::contract {

nlohmann::json query(zenoh::Session &session, const std::string &key,
                     const std::optional<nlohmann::json> &payload,
                     std::chrono::milliseconds timeout) {
  if (timeout.count() < 1 || timeout.count() > 60000) {
    throw std::invalid_argument("query timeout must be 1 to 60000 ms");
  }
  struct Result {
    std::promise<nlohmann::json> promise;
    std::atomic<bool> answered = false;
  };
  auto result = std::make_shared<Result>();
  auto future = result->promise.get_future();
  auto options = zenoh::Session::GetOptions::create_default();
  options.timeout_ms = timeout.count();
  if (payload) {
    options.payload = zenoh::Bytes(payload->dump());
    options.encoding = zenoh::Encoding("application/json");
  }
  session.get(
      zenoh::KeyExpr(key), "",
      [result](zenoh::Reply &reply) {
        if (reply.is_ok() && !result->answered.exchange(true)) {
          try {
            result->promise.set_value(nlohmann::json::parse(
                reply.get_ok().get_payload().as_string()));
          } catch (...) {
            result->promise.set_exception(std::current_exception());
          }
        }
      },
      [result] {
        if (!result->answered.exchange(true)) {
          result->promise.set_exception(
              std::make_exception_ptr(std::runtime_error("no stack replied")));
        }
      },
      std::move(options));
  if (future.wait_for(timeout + std::chrono::seconds(1)) !=
      std::future_status::ready) {
    throw std::runtime_error("query timed out");
  }
  return future.get();
}

void publish(zenoh::Session &session, const std::string &key,
             const nlohmann::json &payload) {
  auto options = zenoh::Session::PutOptions::create_default();
  options.encoding = zenoh::Encoding("application/json");
  session.put(zenoh::KeyExpr(key), zenoh::Bytes(payload.dump()),
              std::move(options));
}

}
