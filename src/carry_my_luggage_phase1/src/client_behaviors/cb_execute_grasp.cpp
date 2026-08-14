#include "carry_my_luggage_phase1/client_behaviors/cb_execute_grasp.hpp"

#include "carry_my_luggage_phase1/events/ev_retry_limit_exceeded.hpp"
#include "carry_my_luggage_phase1/retry_policy.hpp"

#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <future>

using namespace std::chrono_literals;

namespace
{
constexpr auto kGraspResponseTimeout = std::chrono::seconds(60);
constexpr auto kRetryDelay = std::chrono::seconds(2);
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbExecuteGrasp::onEntry()
{
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
  auto client = node->create_client<std_srvs::srv::Trigger>("/grasp/execute");

  if (!client->wait_for_service(10s)){
    RCLCPP_ERROR(getLogger(), "/grasp/execute service is not available.");
    rclcpp::sleep_for(kRetryDelay);
    postRetryOutcome();
    return;
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postRetryOutcome();
    return;
  }

  // This state executes grasp directly; startup readiness is handled in StStartupBringup.
  RCLCPP_INFO(getLogger(), "Calling /grasp/execute directly.");

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  const auto start = std::chrono::steady_clock::now();

  while (!isShutdownRequested() && rclcpp::ok() && future.wait_for(200ms) != std::future_status::ready)
  {
    if ((std::chrono::steady_clock::now() - start) > kGraspResponseTimeout) {
      RCLCPP_ERROR(getLogger(), "Timed out waiting for /grasp/execute response.");
      rclcpp::sleep_for(kRetryDelay);
      postRetryOutcome();
      return;
    }
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postRetryOutcome();
    return;
  }

  const auto response = future.get();
  if (response != nullptr && response->success) {
    // /grasp/execute success means the grasp task completed.
    RCLCPP_INFO(getLogger(), "/grasp/execute returned success: %s", response->message.c_str());
    getStateMachine()->setGlobalSMData("retry_count", 0);
    postSuccessEvent();
    return;
  }

  const auto message = response != nullptr ? response->message : std::string("null response");
  RCLCPP_WARN(getLogger(), "/grasp/execute failed: %s", message.c_str());
  rclcpp::sleep_for(kRetryDelay);
  postRetryOutcome();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
