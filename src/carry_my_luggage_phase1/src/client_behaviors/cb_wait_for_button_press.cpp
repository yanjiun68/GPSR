#include "carry_my_luggage_phase1/client_behaviors/cb_wait_for_button_press.hpp"

#include <std_msgs/msg/bool.hpp>

#include <atomic>
#include <chrono>

using namespace std::chrono_literals;

namespace
{
constexpr char kButtonTopic[] = "/dio13_state";
constexpr auto kPollInterval = std::chrono::milliseconds(100);
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbWaitForButtonPress::onEntry()
{
  auto node = getNode();
  auto logger = getLogger();
  std::atomic_bool pressed{false};

  auto subscription = node->create_subscription<std_msgs::msg::Bool>(
    kButtonTopic,
    10,
    [&pressed, logger](const std_msgs::msg::Bool::SharedPtr msg)
    {
      if (msg != nullptr && msg->data) {
        RCLCPP_INFO(logger, "%s is true; continuing state machine.", kButtonTopic);
        pressed.store(true);
      }
    });
  (void)subscription;

  RCLCPP_INFO(getLogger(), "Waiting for button press on %s.", kButtonTopic);
  while (!isShutdownRequested() && rclcpp::ok() && !pressed.load()) {
    rclcpp::sleep_for(kPollInterval);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postFailureEvent();
    return;
  }

  postSuccessEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
