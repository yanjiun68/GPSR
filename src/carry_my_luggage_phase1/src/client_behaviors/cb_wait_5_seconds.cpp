#include "carry_my_luggage_phase1/client_behaviors/cb_wait_5_seconds.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace
{
constexpr auto kWaitDuration = std::chrono::seconds(5);
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbWait5Seconds::onEntry()
{
  const auto start = std::chrono::steady_clock::now();
  while (!isShutdownRequested() && rclcpp::ok() &&
    (std::chrono::steady_clock::now() - start) < kWaitDuration)
  {
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postFailureEvent();
    return;
  }

  postSuccessEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
