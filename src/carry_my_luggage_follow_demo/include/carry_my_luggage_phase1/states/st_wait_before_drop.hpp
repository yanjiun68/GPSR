#pragma once

#include <boost/mpl/list.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_wait_5_seconds.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_terminal.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"

namespace carry_my_luggage_phase1
{
struct StAborted;
struct StArmRest;

struct StWaitBeforeDrop : smacc2::SmaccState<StWaitBeforeDrop, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<
        client_behaviors::CbWait5Seconds,
        orthogonals::OrTerminal>,
      StArmRest,
      smacc2::SUCCESS>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbWait5Seconds,
        orthogonals::OrTerminal>,
      StAborted,
      smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrTerminal,
      client_behaviors::CbWait5Seconds>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "Entering StWaitBeforeDrop");
  }
};
}  // namespace carry_my_luggage_phase1
