#pragma once

#include <smacc2/smacc.hpp>

#include <chrono>

#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"
#include "carry_my_luggage_phase1/utils/bringup_shutdown.hpp"

namespace carry_my_luggage_phase1
{
using namespace std::chrono_literals;

struct StUltimateFailsafe : smacc2::SmaccState<StUltimateFailsafe, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  static void staticConfigure() {}

  void onEntry()
  {
    RCLCPP_ERROR(
      getLogger(),
      "Entering StUltimateFailsafe. Sending SIGINT-first shutdown to tracked bringup processes.");

    while (rclcpp::ok() && !utils::shutdownHeavyBringupNodes(this)) {
      RCLCPP_ERROR(
        getLogger(),
        "Heavy nodes are still running. Retrying shutdown before ending StUltimateFailsafe.");
      rclcpp::sleep_for(500ms);
    }

    getStateMachine().stop();
  }
};
}  // namespace carry_my_luggage_phase1
