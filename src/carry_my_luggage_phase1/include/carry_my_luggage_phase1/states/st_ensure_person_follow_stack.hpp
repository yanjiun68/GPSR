#pragma once

#include <boost/mpl/list.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_start_person_follow_stack_managed.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_follow.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"

namespace carry_my_luggage_phase1
{
struct StAborted;
struct StFollowOperator;

struct StEnsurePersonFollowStack : smacc2::SmaccState<StEnsurePersonFollowStack, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<
        client_behaviors::CbStartPersonFollowStackManaged,
        orthogonals::OrFollow>,
      StFollowOperator,
      smacc2::SUCCESS>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbStartPersonFollowStackManaged,
        orthogonals::OrFollow>,
      StAborted,
      smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrFollow,
      client_behaviors::CbStartPersonFollowStackManaged>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "Entering StEnsurePersonFollowStack");
  }
};
}  // namespace carry_my_luggage_phase1
