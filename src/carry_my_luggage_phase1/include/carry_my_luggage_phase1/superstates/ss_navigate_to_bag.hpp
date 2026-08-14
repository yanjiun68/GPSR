#pragma once

#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"

namespace carry_my_luggage_phase1
{
struct StBringupBag;

struct SsNavigateToBag
: smacc2::SmaccState<SsNavigateToBag, SmCarryMyLuggagePhase1, StBringupBag>
{
  using SmaccState::SmaccState;

  void onEntry() { RCLCPP_INFO(getLogger(), "Entering SsNavigateToBag"); }
  void onExit() { RCLCPP_INFO(getLogger(), "Exiting SsNavigateToBag"); }
};
}  // namespace carry_my_luggage_phase1
