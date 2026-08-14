#include "carry_my_luggage_phase1/client_behaviors/cb_ready_follow.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <putrabot_interfaces/action/announce_follow_ready.hpp>

#include <chrono>

using namespace std::chrono_literals;

namespace
{
constexpr auto kAnnounceReadyServerTimeout = std::chrono::seconds(10);
constexpr auto kAnnounceReadyResultTimeout = std::chrono::seconds(30);
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbReadyFollow::onEntry()
{
  using AnnounceFollowReady = putrabot_interfaces::action::AnnounceFollowReady;
  auto node = getNode();
  auto client = rclcpp_action::create_client<AnnounceFollowReady>(node, "/person_follow/announce_ready");

  if (!client->wait_for_action_server(kAnnounceReadyServerTimeout)) {
    RCLCPP_WARN(getLogger(), "/person_follow/announce_ready action server is not available.");
    postFailureEvent();
    return;
  }

  AnnounceFollowReady::Goal goal;
  goal.text = "";
  goal.timeout_sec = 0;

  typename rclcpp_action::Client<AnnounceFollowReady>::SendGoalOptions options;
  options.feedback_callback =
    [logger = getLogger()](
    typename rclcpp_action::ClientGoalHandle<AnnounceFollowReady>::SharedPtr,
    const std::shared_ptr<const AnnounceFollowReady::Feedback> feedback)
    {
      RCLCPP_INFO(
        logger, "/person_follow/announce_ready feedback: session_active=%d safety_ok=%d "
                "target_locked=%d speaking=%d waiting_on=%s",
        feedback->session_active, feedback->safety_ok, feedback->target_locked, feedback->speaking,
        feedback->waiting_on.c_str());
    };

  auto goal_future = client->async_send_goal(goal, options);
  while (!isShutdownRequested() && rclcpp::ok() && goal_future.wait_for(100ms) != std::future_status::ready)
  {
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postFailureEvent();
    return;
  }

  auto goal_handle = goal_future.get();
  if (goal_handle == nullptr) {
    RCLCPP_WARN(getLogger(), "/person_follow/announce_ready goal was rejected.");
    postFailureEvent();
    return;
  }

  auto result_future = client->async_get_result(goal_handle);
  const auto start = std::chrono::steady_clock::now();
  while (!isShutdownRequested() && rclcpp::ok() &&
    result_future.wait_for(100ms) != std::future_status::ready)
  {
    if ((std::chrono::steady_clock::now() - start) > kAnnounceReadyResultTimeout) {
      RCLCPP_WARN(getLogger(), "/person_follow/announce_ready timed out.");
      postFailureEvent();
      return;
    }
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postFailureEvent();
    return;
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(
      getLogger(), "/person_follow/announce_ready succeeded: %s", result.result->message.c_str());
    postSuccessEvent();
    return;
  }

  RCLCPP_WARN(getLogger(), "/person_follow/announce_ready finished with non-success result.");
  postFailureEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
