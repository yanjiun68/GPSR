#pragma once

#include <boost/mpl/list.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_navigate_to_waypoint.hpp"
#include "carry_my_luggage_phase1/events/ev_retry_limit_exceeded.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_follow.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"

namespace carry_my_luggage_phase1
{
struct StAborted;
struct StWaitBeforeDrop;

struct StGotoDestination : smacc2::SmaccState<StGotoDestination, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<
        client_behaviors::CbNavigateToWaypoint,
        orthogonals::OrFollow>,
      StWaitBeforeDrop,
      smacc2::SUCCESS>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbNavigateToWaypoint,
        orthogonals::OrFollow>,
      StGotoDestination,
      smacc2::ABORT>,
    smacc2::Transition<
      events::EvRetryLimitExceeded,
      StAborted,
      smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrFollow,
      client_behaviors::CbNavigateToWaypoint>("0");
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "Entering StGotoDestination"); }
};
}  // namespace carry_my_luggage_phase1
