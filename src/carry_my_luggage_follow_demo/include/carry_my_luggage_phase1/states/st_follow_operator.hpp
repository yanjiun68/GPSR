#pragma once

#include <boost/mpl/list.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_start_person_follow_session.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_follow.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"

namespace carry_my_luggage_phase1
{
struct StAborted;
struct StAskPassBagConfirmation;

struct StFollowOperator : smacc2::SmaccState<StFollowOperator, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<
        client_behaviors::CbStartPersonFollowSession,
        orthogonals::OrFollow>,
      StAskPassBagConfirmation,
      smacc2::SUCCESS>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbStartPersonFollowSession,
        orthogonals::OrFollow>,
      StAborted,
      smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrFollow,
      client_behaviors::CbStartPersonFollowSession>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "Entering StFollowOperator");
  }
};
}  // namespace carry_my_luggage_phase1
