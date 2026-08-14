#include "carry_my_luggage_phase1/client_behaviors/cb_start_person_follow_session.hpp"

#include <putrabot_interfaces/action/person_follow.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <future>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

namespace
{
constexpr auto kServerTimeout = std::chrono::seconds(10);
constexpr auto kSttResponseTimeout = std::chrono::seconds(10);
constexpr float kDefaultFollowDistanceM = 0.8F;
constexpr float kDefaultFaceHoldSec = 5.0F;

struct SpeechMonitorState
{
  bool last_facing_robot = false;
  bool stt_request_pending = false;
  bool stop_command_heard = false;
  std::chrono::steady_clock::time_point stt_triggered_at{};
};

std::string toLower(std::string text)
{
  std::transform(
    text.begin(), text.end(), text.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return text;
}

bool containsStopKeyword(const std::string & transcript)
{
  std::string normalized = toLower(transcript);
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char c) {return std::isalpha(c) ? static_cast<char>(c) : ' ';});

  return normalized == "stop" || normalized.find(" stop ") != std::string::npos ||
         normalized.rfind("stop ", 0) == 0 ||
         (normalized.size() >= 5 && normalized.compare(normalized.size() - 5, 5, " stop") == 0);
}
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbStartPersonFollowSession::onEntry()
{
  using PersonFollow = putrabot_interfaces::action::PersonFollow;

  auto node = getNode();
  auto client = rclcpp_action::create_client<PersonFollow>(node, "/person_follow/control");
  auto speech_state = std::make_shared<SpeechMonitorState>();
  auto speech_mutex = std::make_shared<std::mutex>();
  auto stt_trigger_pub = node->create_publisher<std_msgs::msg::Bool>("/voice/stt/start", 10);
  auto transcript_sub = node->create_subscription<std_msgs::msg::String>(
    "/voice/stt/transcript", 10,
    [logger = getLogger(), speech_state, speech_mutex](const std_msgs::msg::String::SharedPtr msg)
    {
      const std::string transcript = msg != nullptr ? msg->data : "";

      std::lock_guard<std::mutex> lock(*speech_mutex);
      if (!speech_state->stt_request_pending) {
        return;
      }
      speech_state->stt_request_pending = false;

      if (containsStopKeyword(transcript)) {
        speech_state->stop_command_heard = true;
        RCLCPP_INFO(logger, "Detected spoken stop command from transcript: %s", transcript.c_str());
        return;
      }

      RCLCPP_INFO(logger, "STT transcript did not contain stop: %s", transcript.c_str());
    });
  (void)transcript_sub;

  if (!client->wait_for_action_server(kServerTimeout)) {
    RCLCPP_WARN(getLogger(), "/person_follow/control action server is not available.");
    postFailureEvent();
    return;
  }

  PersonFollow::Goal goal;
  goal.auto_select_if_needed = true;
  goal.follow_distance_m = kDefaultFollowDistanceM;
  goal.face_hold_sec = kDefaultFaceHoldSec;

  typename rclcpp_action::Client<PersonFollow>::SendGoalOptions options;
  options.feedback_callback =
    [logger = getLogger(), stt_trigger_pub, speech_state, speech_mutex](
    typename rclcpp_action::ClientGoalHandle<PersonFollow>::SharedPtr,
    const std::shared_ptr<const PersonFollow::Feedback> feedback)
    {
      RCLCPP_INFO(
        logger,
        "/person_follow/control feedback: active=%d target_visible=%d facing_robot=%d stop_reason=%s",
        feedback->active,
        feedback->target_visible,
        feedback->facing_robot,
        feedback->stop_reason.c_str());

      bool should_trigger_stt = false;
      {
        std::lock_guard<std::mutex> lock(*speech_mutex);
        if (feedback->facing_robot && !speech_state->last_facing_robot &&
          !speech_state->stt_request_pending && !speech_state->stop_command_heard)
        {
          speech_state->stt_request_pending = true;
          speech_state->stt_triggered_at = std::chrono::steady_clock::now();
          should_trigger_stt = true;
        }
        speech_state->last_facing_robot = feedback->facing_robot;
      }

      if (should_trigger_stt) {
        std_msgs::msg::Bool trigger_msg;
        trigger_msg.data = true;
        stt_trigger_pub->publish(trigger_msg);
        RCLCPP_INFO(logger, "Target is facing robot. Triggered STT on /voice/stt/start.");
      }
    };

  auto goal_future = client->async_send_goal(goal, options);
  while (!isShutdownRequested() && rclcpp::ok() && goal_future.wait_for(100ms) != std::future_status::ready) {
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
  bool cancel_requested_for_stop = false;
  while (!isShutdownRequested() && rclcpp::ok() &&
    result_future.wait_for(100ms) != std::future_status::ready)
  {
    bool stt_timed_out = false;
    bool stop_command_heard = false;
    {
      std::lock_guard<std::mutex> lock(*speech_mutex);
      if (speech_state->stt_request_pending &&
        (std::chrono::steady_clock::now() - speech_state->stt_triggered_at) > kSttResponseTimeout)
      {
        speech_state->stt_request_pending = false;
        stt_timed_out = true;
      }
      stop_command_heard = speech_state->stop_command_heard;
    }

    if (stt_timed_out) {
      RCLCPP_WARN(getLogger(), "Timed out waiting for /voice/stt/transcript after facing trigger.");
    }

    if (stop_command_heard && !cancel_requested_for_stop) {
      cancel_requested_for_stop = true;
      RCLCPP_INFO(getLogger(), "Stop command heard. Canceling /person_follow/control goal.");
      client->async_cancel_goal(goal_handle);
    }

    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postFailureEvent();
    return;
  }

  auto result = result_future.get();
  if (result.result == nullptr) {
    RCLCPP_WARN(getLogger(), "/person_follow/control returned an empty result.");
    postFailureEvent();
    return;
  }

  const std::string message = result.result->message;
  {
    std::lock_guard<std::mutex> lock(*speech_mutex);
    if (speech_state->stop_command_heard) {
      RCLCPP_INFO(
        getLogger(), "Operator stop command detected during person-follow session: code=%d message=%s",
        static_cast<int>(result.code), message.c_str());
      postSuccessEvent();
      return;
    }
  }

  if (message.find("behavior_aborted") != std::string::npos) {
    RCLCPP_INFO(getLogger(), "Operator arrival detected from person-follow result: %s", message.c_str());
    postSuccessEvent();
    return;
  }

  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result->success) {
    RCLCPP_WARN(
      getLogger(), "/person_follow/control failed to complete: code=%d message=%s",
      static_cast<int>(result.code), message.c_str());
    postFailureEvent();
    return;
  }

  RCLCPP_WARN(getLogger(), "/person_follow/control ended without arrival condition: %s", message.c_str());
  postFailureEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
