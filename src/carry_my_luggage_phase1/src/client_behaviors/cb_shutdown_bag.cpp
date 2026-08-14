#include "carry_my_luggage_phase1/client_behaviors/cb_shutdown_bag.hpp"

#include "carry_my_luggage_phase1/events/ev_retry_limit_exceeded.hpp"
#include "carry_my_luggage_phase1/retry_policy.hpp"

#include <putrabot_interfaces/srv/kill_managed_process.hpp>

#include <chrono>
#include <future>

using namespace std::chrono_literals;

namespace
{
constexpr char kKillServiceName[] = "/kill";
constexpr char kBagProcessTarget[] = "bag_point_selection";
constexpr auto kServerTimeout = std::chrono::seconds(10);
constexpr auto kKillResponseTimeout = std::chrono::seconds(15);
constexpr auto kRetryDelay = std::chrono::seconds(2);

bool isAlreadyStoppedMessage(const std::string & message)
{
  return message.find("not running") != std::string::npos ||
         message.find("no active launches") != std::string::npos;
}
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbShutdownBag::onEntry()
{
  using KillManagedProcess = putrabot_interfaces::srv::KillManagedProcess;

  auto postRetryOutcome = [this]()
    {
      int retry_count = 0;
      getStateMachine()->getGlobalSMData("retry_count", retry_count);
      retry_count++;
      getStateMachine()->setGlobalSMData("retry_count", retry_count);

      if (retry_count > kMaxRetryAttempts) {
        postEvent<events::EvRetryLimitExceeded>();
      } else {
        postFailureEvent();
      }
    };

  auto node = getNode();
  auto client = node->create_client<KillManagedProcess>(kKillServiceName);

  if (!client->wait_for_service(kServerTimeout)) {
    RCLCPP_WARN(getLogger(), "%s service is not available.", kKillServiceName);
    rclcpp::sleep_for(kRetryDelay);
    postRetryOutcome();
    return;
  }

  auto request = std::make_shared<KillManagedProcess::Request>();
  request->target = kBagProcessTarget;

  auto future = client->async_send_request(request);
  const auto start = std::chrono::steady_clock::now();
  while (!isShutdownRequested() && rclcpp::ok() &&
    future.wait_for(100ms) != std::future_status::ready)
  {
    if ((std::chrono::steady_clock::now() - start) > kKillResponseTimeout) {
      RCLCPP_WARN(getLogger(), "%s timed out while stopping %s.", kKillServiceName, kBagProcessTarget);
      rclcpp::sleep_for(kRetryDelay);
      postRetryOutcome();
      return;
    }
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postRetryOutcome();
    return;
  }

  const auto response = future.get();
  if (response != nullptr && (response->success || isAlreadyStoppedMessage(response->message))) {
    RCLCPP_INFO(getLogger(), "Stopped %s: %s", kBagProcessTarget, response->message.c_str());
    getStateMachine()->setGlobalSMData("retry_count", 0);
    postSuccessEvent();
    return;
  }

  const auto message = response != nullptr ? response->message : std::string("null response");
  RCLCPP_WARN(getLogger(), "Failed to stop %s: %s", kBagProcessTarget, message.c_str());
  rclcpp::sleep_for(kRetryDelay);
  postRetryOutcome();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
