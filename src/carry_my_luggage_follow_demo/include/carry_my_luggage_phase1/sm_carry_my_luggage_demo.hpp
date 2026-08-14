#pragma once

#include <cstdint>

#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/orthogonals/or_bringup.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_follow.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_manipulation.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_terminal.hpp"

namespace carry_my_luggage_phase1
{
struct StStartupBringup;

struct SmCarryMyLuggagePhase1
: smacc2::SmaccStateMachineBase<SmCarryMyLuggagePhase1, StStartupBringup>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<orthogonals::OrBringup>();
    this->createOrthogonal<orthogonals::OrFollow>();
    this->createOrthogonal<orthogonals::OrManipulation>();
    this->createOrthogonal<orthogonals::OrTerminal>();
    this->setGlobalSMData("retry_count", 0);
    this->setGlobalSMData("bringup_pid", static_cast<std::int64_t>(-1));

    RCLCPP_INFO(
      getLogger(),
      "carry_my_luggage_phase1 initialized. The state machine now runs a combined "
      "phase1 + phase2 carry-my-luggage flow.");
  }
};
}  // namespace carry_my_luggage_phase1
