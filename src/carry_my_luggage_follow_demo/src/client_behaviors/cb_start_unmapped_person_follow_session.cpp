#include "carry_my_luggage_phase1/client_behaviors/cb_start_unmapped_person_follow_session.hpp"

#include <putrabot_interfaces/action/person_follow.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <future>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace
{
constexpr auto kModeSubscriberTimeout = std::chrono::seconds(3);
constexpr auto kActionServerTimeout = std::chrono::seconds(10);
constexpr float kFollowDistanceM = 0.8F;
constexpr float kFaceHoldSec = 10.0F;

void publishUnmappedMode(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Logger & logger)
{
  auto publisher = node->create_publisher<std_msgs::msg::String>("/person_follow/mode", 10);
  const auto wait_start = std::chrono::steady_clock::now();

  while (rclcpp::ok() &&
    publisher->get_subscription_count() == 0 &&
    (std::chrono::steady_clock::now() - wait_start) < kModeSubscriberTimeout)
  {
    rclcpp::sleep_for(100ms);
  }

  if (publisher->get_subscription_count() == 0) {
    RCLCPP_WARN(
      logger,
      "Publishing /person_follow/mode='unmapped' with no matched subscribers yet.");
  }

  std_msgs::msg::String mode_msg;
  mode_msg.data = "unmapped";
  publisher->publish(mode_msg);
  RCLCPP_INFO(logger, "Published /person_follow/mode='unmapped'.");
  rclcpp::sleep_for(200ms);
}
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbStartUnmappedPersonFollowSession::onEntry()
{
  using PersonFollow = putrabot_interfaces::action::PersonFollow;

  auto node = getNode();
  publishUnmappedMode(node, getLogger());

  auto client = rclcpp_action::create_client<PersonFollow>(node, "/person_follow/control");
  if (!client->wait_for_action_server(kActionServerTimeout)) {
    RCLCPP_WARN(getLogger(), "/person_follow/control action server is not available.");
    postFailureEvent();
    return;
  }

  PersonFollow::Goal goal;
  goal.auto_select_if_needed = true;
  goal.follow_distance_m = kFollowDistanceM;
  goal.face_hold_sec = kFaceHoldSec;

  typename rclcpp_action::Client<PersonFollow>::SendGoalOptions options;
  options.feedback_callback =
    [logger = getLogger()](
    typename rclcpp_action::ClientGoalHandle<PersonFollow>::SharedPtr,
    const std::shared_ptr<const PersonFollow::Feedback> feedback)
    {
      RCLCPP_INFO(
        logger,
        "/person_follow/control feedback: active=%d selected_track_id=%d "
        "target_visible=%d facing_robot=%d stop_reason=%s",
        feedback->active,
        feedback->selected_track_id,
        feedback->target_visible,
        feedback->facing_robot,
        feedback->stop_reason.c_str());
    };

  RCLCPP_INFO(
    getLogger(),
    "Sending /person_follow/control goal: auto_select_if_needed=true, "
    "follow_distance_m=%.2f, face_hold_sec=%.2f.",
    static_cast<double>(kFollowDistanceM),
    static_cast<double>(kFaceHoldSec));

  auto goal_future = client->async_send_goal(goal, options);
  while (!isShutdownRequested() && rclcpp::ok() &&
    goal_future.wait_for(100ms) != std::future_status::ready)
  {
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postFailureEvent();
    return;
  }

  auto goal_handle = goal_future.get();
  if (goal_handle == nullptr) {
    RCLCPP_WARN(getLogger(), "/person_follow/control goal was rejected.");
    postFailureEvent();
    return;
  }

  auto result_future = client->async_get_result(goal_handle);
  while (!isShutdownRequested() && rclcpp::ok() &&
    result_future.wait_for(100ms) != std::future_status::ready)
  {
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    client->async_cancel_goal(goal_handle);
    postFailureEvent();
    return;
  }

  const auto result = result_future.get();
  if (result.result == nullptr) {
    RCLCPP_WARN(getLogger(), "/person_follow/control returned an empty result.");
    postFailureEvent();
    return;
  }

  const std::string message = result.result->message;
  if (message.find("behavior_aborted") != std::string::npos) {
    RCLCPP_INFO(getLogger(), "Destination arrival detected from person-follow result: %s", message.c_str());
    postSuccessEvent();
    return;
  }

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
    RCLCPP_INFO(getLogger(), "/person_follow/control succeeded: %s", message.c_str());
    postSuccessEvent();
    return;
  }

  RCLCPP_WARN(
    getLogger(),
    "/person_follow/control failed to complete destination leg: code=%d success=%d message=%s",
    static_cast<int>(result.code),
    result.result->success,
    message.c_str());
  postFailureEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
