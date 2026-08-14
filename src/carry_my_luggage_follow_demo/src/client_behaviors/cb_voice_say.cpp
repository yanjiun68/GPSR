#include "carry_my_luggage_phase1/client_behaviors/cb_voice_say.hpp"

#include <voice_interfaces/srv/say_text.hpp>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <utility>

using namespace std::chrono_literals;

namespace carry_my_luggage_phase1::client_behaviors
{
CbVoiceSay::CbVoiceSay(std::string text, std::string service_name, double timeout_sec)
: text_(std::move(text)),
  service_name_(std::move(service_name)),
  timeout_sec_(timeout_sec)
{
}

void CbVoiceSay::onEntry()
{
  using SayText = voice_interfaces::srv::SayText;

  auto node = getNode();
  std::string text = text_;
  if (text.empty()) {
    getStateMachine()->getGlobalSMData("voice_say_text", text);
  }

  if (text.empty()) {
    RCLCPP_WARN(getLogger(), "CbVoiceSay has no text to speak.");
    postFailureEvent();
    return;
  }

  const auto timeout =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(timeout_sec_));

  auto client = node->create_client<SayText>(service_name_);
  if (!client->wait_for_service(timeout)) {
    RCLCPP_WARN(getLogger(), "%s service is not available.", service_name_.c_str());
    postFailureEvent();
    return;
  }

  auto request = std::make_shared<SayText::Request>();
  request->text = text;
  auto future = client->async_send_request(request);

  const auto start = std::chrono::steady_clock::now();
  while (!isShutdownRequested() && rclcpp::ok() &&
    future.wait_for(100ms) != std::future_status::ready)
  {
    if ((std::chrono::steady_clock::now() - start) > timeout) {
      RCLCPP_WARN(getLogger(), "%s timed out.", service_name_.c_str());
      postFailureEvent();
      return;
    }
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postFailureEvent();
    return;
  }

  auto response = future.get();
  if (response == nullptr || !response->success) {
    RCLCPP_WARN(
      getLogger(), "%s failed: %s",
      service_name_.c_str(),
      response ? response->message.c_str() : "no response");
    postFailureEvent();
    return;
  }

  postSuccessEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
