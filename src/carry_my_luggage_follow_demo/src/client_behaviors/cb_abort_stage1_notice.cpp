#include "carry_my_luggage_phase1/client_behaviors/cb_abort_stage1_notice.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace carry_my_luggage_phase1::client_behaviors
{
void CbAbortStage1Notice::onEntry()
{
  const auto start = std::chrono::steady_clock::now();

  while (!isShutdownRequested() && rclcpp::ok() && (std::chrono::steady_clock::now() - start) < 10s)
  {
    RCLCPP_INFO(getLogger(), "abort stage 1");
    rclcpp::sleep_for(1s);
  }

  postSuccessEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
