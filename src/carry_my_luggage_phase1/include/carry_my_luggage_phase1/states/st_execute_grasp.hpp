#pragma once

#include <boost/mpl/list.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_execute_grasp.hpp"
#include "carry_my_luggage_phase1/events/ev_retry_limit_exceeded.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_manipulation.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"

namespace carry_my_luggage_phase1
{
// struct StAborted;
struct StFollowOperator;
struct StKillRx200Camera;

struct StExecuteGrasp : smacc2::SmaccState<StExecuteGrasp, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<
        client_behaviors::CbExecuteGrasp,
        orthogonals::OrManipulation>,
      StKillRx200Camera,
      smacc2::SUCCESS>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbExecuteGrasp,
        orthogonals::OrManipulation>,
      StExecuteGrasp,
      smacc2::ABORT>,
    smacc2::Transition<
      events::EvRetryLimitExceeded,
      StKillRx200Camera,
      smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrManipulation,
      client_behaviors::CbExecuteGrasp>();
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "Entering StExecuteGrasp"); }
};
}  // namespace carry_my_luggage_phase1
