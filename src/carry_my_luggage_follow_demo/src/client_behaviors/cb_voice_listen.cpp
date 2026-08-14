#include "carry_my_luggage_phase1/client_behaviors/cb_voice_listen.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <voice_interfaces/action/transcribe_speech.hpp>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <utility>

using namespace std::chrono_literals;

namespace carry_my_luggage_phase1::client_behaviors
{
CbVoiceListen::CbVoiceListen(
  std::string transcript_key,
  float record_sec,
  std::string mic_device,
  std::string whisper_model,
  std::string whisper_language,
  std::string action_name,
  std::string wav_path)
: transcript_key_(std::move(transcript_key)),
  record_sec_(record_sec),
  mic_device_(std::move(mic_device)),
  whisper_model_(std::move(whisper_model)),
  whisper_language_(std::move(whisper_language)),
  action_name_(std::move(action_name)),
  wav_path_(std::move(wav_path))
{
}

void CbVoiceListen::onEntry()
{
  using TranscribeSpeech = voice_interfaces::action::TranscribeSpeech;

  auto node = getNode();
  auto client = rclcpp_action::create_client<TranscribeSpeech>(node, action_name_);
  const auto server_timeout =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(server_timeout_sec_));
  const auto result_timeout =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(result_timeout_sec_));

  auto wait_for_future = [this](auto & future, const std::chrono::steady_clock::duration timeout)
  {
    const auto start = std::chrono::steady_clock::now();
    while (!isShutdownRequested() && rclcpp::ok() &&
      future.wait_for(100ms) != std::future_status::ready)
    {
      if ((std::chrono::steady_clock::now() - start) > timeout) {
        return false;
      }
      rclcpp::sleep_for(100ms);
    }
    return rclcpp::ok() && !isShutdownRequested();
  };

  if (!client->wait_for_action_server(server_timeout)) {
    RCLCPP_WARN(getLogger(), "%s action server is not available.", action_name_.c_str());
    postFailureEvent();
    return;
  }

  TranscribeSpeech::Goal goal;
  goal.record_sec = record_sec_;
  goal.mic_device = mic_device_;
  goal.whisper_model = whisper_model_;
  goal.whisper_language = whisper_language_;
  goal.wav_path = wav_path_;

  typename rclcpp_action::Client<TranscribeSpeech>::SendGoalOptions options;
  options.feedback_callback =
    [logger = getLogger()](
    typename rclcpp_action::ClientGoalHandle<TranscribeSpeech>::SharedPtr,
    const std::shared_ptr<const TranscribeSpeech::Feedback> feedback)
    {
      RCLCPP_INFO(
        logger, "/voice/stt/listen feedback: stage=%s message=%s",
        feedback->stage.c_str(), feedback->message.c_str());
    };

  auto goal_future = client->async_send_goal(goal, options);
  if (!wait_for_future(goal_future, server_timeout)) {
    RCLCPP_WARN(getLogger(), "%s goal timed out.", action_name_.c_str());
    postFailureEvent();
    return;
  }

  auto goal_handle = goal_future.get();
  if (goal_handle == nullptr) {
    RCLCPP_WARN(getLogger(), "%s goal was rejected.", action_name_.c_str());
    postFailureEvent();
    return;
  }

  auto result_future = client->async_get_result(goal_handle);
  if (!wait_for_future(result_future, result_timeout)) {
    RCLCPP_WARN(getLogger(), "%s result timed out.", action_name_.c_str());
    postFailureEvent();
    return;
  }

  auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
    !wrapped_result.result->success)
  {
    RCLCPP_WARN(
      getLogger(), "%s failed: %s",
      action_name_.c_str(),
      wrapped_result.result ? wrapped_result.result->message.c_str() : "no result");
    postFailureEvent();
    return;
  }

  getStateMachine()->setGlobalSMData(transcript_key_, wrapped_result.result->transcript);
  getStateMachine()->setGlobalSMData("voice_last_transcript", wrapped_result.result->transcript);
  postSuccessEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
