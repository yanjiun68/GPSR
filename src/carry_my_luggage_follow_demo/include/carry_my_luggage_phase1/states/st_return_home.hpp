#pragma once

#include <boost/mpl/list.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_return_home_managed.hpp"
// #include "carry_my_luggage_phase1/client_behaviors/cb_navigate_to_waypoint.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_follow.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"

namespace carry_my_luggage_phase1
{
struct StAborted;
struct StAbortAllProcesses;

struct StReturnHome : smacc2::SmaccState<StReturnHome, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<
        client_behaviors::CbReturnHomeManaged,
        orthogonals::OrFollow>,
      StAbortAllProcesses,
      smacc2::SUCCESS>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbReturnHomeManaged,
        orthogonals::OrFollow>,
      StAborted,
      smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrFollow,
      client_behaviors::CbReturnHomeManaged>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "Entering StReturnHome");
  }
};
}  // namespace carry_my_luggage_phase1
