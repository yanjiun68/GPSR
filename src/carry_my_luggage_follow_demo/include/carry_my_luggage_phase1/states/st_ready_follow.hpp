#pragma once

#include <boost/mpl/list.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_ready_follow.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_voice.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"

namespace carry_my_luggage_phase1
{
struct StAborted;
struct StEnsurePersonFollowStack;

struct StReadyFollow : smacc2::SmaccState<StReadyFollow, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<
        client_behaviors::CbReadyFollow,
        orthogonals::OrVoice>,
      StEnsurePersonFollowStack,
      smacc2::SUCCESS>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbReadyFollow,
        orthogonals::OrVoice>,
      StAborted,
      smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrVoice,
      client_behaviors::CbReadyFollow>();
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "Entering StReadyFollow"); }
};
}  // namespace carry_my_luggage_phase1
