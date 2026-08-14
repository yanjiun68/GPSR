#include "carry_my_luggage_phase1/client_behaviors/cb_select_bag.hpp"

#include <std_msgs/msg/bool.hpp>

#include <atomic>
#include <chrono>

using namespace std::chrono_literals;

namespace
{
constexpr char kBagReadyTopic[] = "/bag_ready_for_pickup";
constexpr auto kPublisherWaitTimeout = std::chrono::seconds(10);
constexpr auto kPollInterval = std::chrono::milliseconds(100);
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbSelectBag::onEntry()
{
  auto node = getNode();
  std::atomic_bool bag_ready{false};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription;
  try {
    subscription = node->create_subscription<std_msgs::msg::Bool>(
      kBagReadyTopic,
      10,
      [this, &bag_ready](const std_msgs::msg::Bool::SharedPtr msg)
      {
        if (msg != nullptr && msg->data) {
          RCLCPP_INFO(getLogger(), "%s is true; proceeding to shutdown state.", kBagReadyTopic);
          bag_ready.store(true);
        }
      });
  } catch (const std::exception & ex) {
    RCLCPP_WARN(getLogger(), "Failed to subscribe to %s: %s", kBagReadyTopic, ex.what());
    postFailureEvent();
    return;
  }

  if (subscription == nullptr) {
    RCLCPP_WARN(getLogger(), "Failed to create subscription for %s.", kBagReadyTopic);
    postFailureEvent();
    return;
  }

  RCLCPP_INFO(getLogger(), "Waiting for a publisher on %s.", kBagReadyTopic);
  const auto wait_start = std::chrono::steady_clock::now();
  while (!isShutdownRequested() && rclcpp::ok()) {
    if (node->count_publishers(kBagReadyTopic) > 0) {
      break;
    }

    if ((std::chrono::steady_clock::now() - wait_start) > kPublisherWaitTimeout) {
      RCLCPP_WARN(
        getLogger(),
        "No publishers found on %s within %.1f s. Returning to bringup.",
        kBagReadyTopic,
        std::chrono::duration<double>(kPublisherWaitTimeout).count());
      postFailureEvent();
      return;
    }

    rclcpp::sleep_for(kPollInterval);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postFailureEvent();
    return;
  }

  RCLCPP_INFO(getLogger(), "Subscribed to %s. Waiting for true.", kBagReadyTopic);
  while (!isShutdownRequested() && rclcpp::ok() && !bag_ready.load()) {
    rclcpp::sleep_for(kPollInterval);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postFailureEvent();
    return;
  }

  postSuccessEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
