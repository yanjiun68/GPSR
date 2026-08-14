#pragma once

#include <chrono>

#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"
#include "carry_my_luggage_phase1/utils/bringup_shutdown.hpp"

namespace carry_my_luggage_phase1
{
using namespace std::chrono_literals;

struct StCompleted : smacc2::SmaccState<StCompleted, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  static void staticConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "carry_my_luggage_phase1 combined phase sequence completed.");

    while (rclcpp::ok() && !utils::shutdownHeavyBringupNodes(this)) {
      RCLCPP_ERROR(
        getLogger(),
        "Heavy nodes are still running. Retrying shutdown before ending StCompleted.");
      rclcpp::sleep_for(500ms);
    }

    getStateMachine().stop();
  }
};
}  // namespace carry_my_luggage_phase1
