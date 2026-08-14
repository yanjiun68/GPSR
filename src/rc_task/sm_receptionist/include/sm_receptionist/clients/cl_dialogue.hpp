#pragma once

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "receptionist_interfaces/action/collect_guest_profile.hpp"

namespace sm_receptionist
{

class ClDialogue
{
public:
  using CollectGuestProfile = receptionist_interfaces::action::CollectGuestProfile;
  using GoalHandle = rclcpp_action::ClientGoalHandle<CollectGuestProfile>;

  explicit ClDialogue(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    // Create the action client used to run the guest dialogue node.
    client_ = rclcpp_action::create_client<CollectGuestProfile>(
      node_,
      "/dialogue/collect_guest_profile");
  }

  bool collectProfile(receptionist_interfaces::msg::PersonProfile & out_profile)
  {
    // Wait briefly for the dialogue action server to appear.
    if (!client_->wait_for_action_server(std::chrono::seconds(3))) {
      return false;
    }

    // Read or declare the microphone device used by the dialogue action.
    const auto mic_device = declareOrGetString("dialogue_mic_device", "default");
    // Read or declare the recording duration used by the dialogue action.
    const auto record_sec = declareOrGetDouble("dialogue_record_sec", 4.0);
    // Read or declare the Piper model path used by the dialogue action.
    const auto piper_model_path = declareOrGetString(
      "dialogue_piper_model_path",
      "/home/yanjiun68/piper_models/en_US-lessac-medium.onnx");
    // Read or declare the Whisper model name used by the dialogue action.
    const auto whisper_model = declareOrGetString("dialogue_whisper_model", "base");
    // Read or declare the Whisper language used by the dialogue action.
    const auto whisper_language = declareOrGetString("dialogue_whisper_language", "en");

    // Build the dialogue action goal using the configured runtime parameters.
    CollectGuestProfile::Goal goal;
    goal.mic_device = mic_device;
    goal.record_sec = static_cast<float>(record_sec);
    goal.piper_model_path = piper_model_path;
    goal.whisper_model = whisper_model;
    goal.whisper_language = whisper_language;

    // Send the goal and wait for goal acceptance.
    auto future_goal = client_->async_send_goal(goal);
    if (future_goal.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      return false;
    }

    // Read the accepted goal handle.
    auto goal_handle = future_goal.get();
    if (!goal_handle) {
      return false;
    }

    // Wait for the action result.
    auto future_result = client_->async_get_result(goal_handle);
    if (future_result.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
      return false;
    }

    // Read the wrapped action result.
    auto wrapped_result = future_result.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      return false;
    }

    // Refuse the result when the action server reported failure.
    if (!wrapped_result.result->success) {
      return false;
    }

    // Return the collected profile to the caller.
    out_profile = wrapped_result.result->profile;
    return true;
  }

private:
  std::string declareOrGetString(const std::string & name, const std::string & default_value)
  {
    // Declare the parameter when it does not exist yet.
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, default_value);
    }

    // Return the current parameter value as a string.
    return node_->get_parameter(name).as_string();
  }

  double declareOrGetDouble(const std::string & name, double default_value)
  {
    // Declare the parameter when it does not exist yet.
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, default_value);
    }

    // Return the current parameter value as a double.
    return node_->get_parameter(name).as_double();
  }

  // Keep one shared node pointer so the action client can be created from it.
  rclcpp::Node::SharedPtr node_;
  // Keep the action client used to collect the guest profile.
  rclcpp_action::Client<CollectGuestProfile>::SharedPtr client_;
};

}  // namespace sm_receptionist
