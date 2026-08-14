#include "carry_my_luggage_phase1/client_behaviors/cb_ready_pose.hpp"

#include "carry_my_luggage_phase1/events/ev_retry_limit_exceeded.hpp"
#include "carry_my_luggage_phase1/retry_policy.hpp"

#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <future>

using namespace std::chrono_literals;

namespace
{
constexpr auto kServiceTimeout = std::chrono::seconds(10);
constexpr auto kResponseTimeout = std::chrono::seconds(60);
constexpr auto kRetryDelay = std::chrono::seconds(1);
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbReadyPose::onEntry()
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
  auto client = node->create_client<std_srvs::srv::Trigger>("/grasp/execute_ready");

  if (!client->wait_for_service(kServiceTimeout)) {
    RCLCPP_WARN(getLogger(), "/grasp/execute_ready service is not available.");
    rclcpp::sleep_for(kRetryDelay);
    postRetryOutcome();
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  const auto start = std::chrono::steady_clock::now();

  while (!isShutdownRequested() && rclcpp::ok() && future.wait_for(100ms) != std::future_status::ready) {
    if ((std::chrono::steady_clock::now() - start) > kResponseTimeout) {
      RCLCPP_WARN(getLogger(), "Timed out waiting for /grasp/execute_ready.");
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
  if (response == nullptr) {
    RCLCPP_WARN(getLogger(), "/grasp/execute_ready returned a null response.");
    rclcpp::sleep_for(kRetryDelay);
    postRetryOutcome();
    return;
  }

  if (response->success) {
    RCLCPP_INFO(getLogger(), "/grasp/execute_ready succeeded: %s", response->message.c_str());
    getStateMachine()->setGlobalSMData("retry_count", 0);
    postSuccessEvent();
    return;
  }

  RCLCPP_WARN(getLogger(), "/grasp/execute_ready failed: %s", response->message.c_str());
  rclcpp::sleep_for(kRetryDelay);
  postRetryOutcome();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
