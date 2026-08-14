#include "carry_my_luggage_phase1/client_behaviors/cb_execute_grasp.hpp"

#include "carry_my_luggage_phase1/events/ev_retry_limit_exceeded.hpp"
#include "carry_my_luggage_phase1/retry_policy.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <future>
#include <mutex>

using namespace std::chrono_literals;

namespace
{
constexpr auto kGraspResponseTimeout = std::chrono::seconds(60);
constexpr auto kHandbagDetectedHoldDuration = std::chrono::seconds(5);
constexpr auto kDetectionWaitTimeout = std::chrono::seconds(45);
constexpr auto kDetectionPollInterval = std::chrono::milliseconds(200);
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
  auto logger = getLogger();
  auto client = node->create_client<std_srvs::srv::Trigger>("/grasp/execute");

  if (!client->wait_for_service(10s)){
    RCLCPP_ERROR(getLogger(), "/grasp/execute service is not available.");
    rclcpp::sleep_for(kRetryDelay);
    postRetryOutcome();
    return;
  }

  std::mutex detection_mutex;
  std::chrono::steady_clock::time_point detection_started_at{};
  bool detection_active = false;

  auto detection_sub = node->create_subscription<std_msgs::msg::Bool>(
    "/handbag/selected", 10,
    [&](const std_msgs::msg::Bool::SharedPtr msg)
    {
      const auto now = std::chrono::steady_clock::now();
      std::lock_guard<std::mutex> lock(detection_mutex);

      if (msg->data) {
        if (!detection_active) {
          detection_active = true;
          detection_started_at = now;
          RCLCPP_INFO(logger, "Handbag candidate detected. Waiting for 5s consecutive lock.");
        }
      } else if (detection_active) {
        detection_active = false;
        RCLCPP_WARN(logger, "Handbag detection lost before 5s consecutive lock.");
      }
    });
  (void)detection_sub;

  const auto detection_wait_start = std::chrono::steady_clock::now();
  while (!isShutdownRequested() && rclcpp::ok()) {
    bool is_consecutive_for_long_enough = false;
    {
      std::lock_guard<std::mutex> lock(detection_mutex);
      if (detection_active) {
        is_consecutive_for_long_enough =
          (std::chrono::steady_clock::now() - detection_started_at) >= kHandbagDetectedHoldDuration;
      }
    }

    if (is_consecutive_for_long_enough) {
      RCLCPP_INFO(logger, "Handbag detected continuously for 5s. Calling /grasp/execute.");
      break;
    }

    if ((std::chrono::steady_clock::now() - detection_wait_start) > kDetectionWaitTimeout) {
      RCLCPP_WARN(logger, "Timed out waiting for 5s consecutive handbag detection.");
      rclcpp::sleep_for(kRetryDelay);
      postRetryOutcome();
      return;
    }

    rclcpp::sleep_for(kDetectionPollInterval);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postRetryOutcome();
    return;
  }

  // This state executes grasp only; startup readiness is handled in StStartupBringup.

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
