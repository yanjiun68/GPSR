#pragma once

#include <smacc2/smacc.hpp>

#include <chrono>

#include "carry_my_luggage_phase1/retry_policy.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"
#include "carry_my_luggage_phase1/utils/bringup_shutdown.hpp"

namespace carry_my_luggage_phase1
{
using namespace std::chrono_literals;

struct StAborted : smacc2::SmaccState<StAborted, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  static void staticConfigure() {}

  void onEntry()
  {
    int retry_count = 0;
    getStateMachine().getGlobalSMData("retry_count", retry_count);
    RCLCPP_ERROR(
      getLogger(),
      "Retry limit exceeded (%d > %d). Aborting carry_my_luggage_phase1 state machine.",
      retry_count, kMaxRetryAttempts);

    while (rclcpp::ok() && !utils::shutdownHeavyBringupNodes(this)) {
      RCLCPP_ERROR(
        getLogger(),
        "Heavy nodes are still running. Retrying shutdown before ending StAborted.");
      rclcpp::sleep_for(500ms);
    }

    getStateMachine().stop();
  }
};
}  // namespace carry_my_luggage_phase1
