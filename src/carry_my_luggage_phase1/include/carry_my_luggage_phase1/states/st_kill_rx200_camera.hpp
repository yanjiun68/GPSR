#pragma once

#include <boost/mpl/list.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_kill_rx200_camera.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_bringup.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"
#include "carry_my_luggage_phase1/states/st_follow_operator.hpp"

namespace carry_my_luggage_phase1
{
struct StKillRx200Camera : smacc2::SmaccState<StKillRx200Camera, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<
        client_behaviors::CbKillRx200Camera,
        orthogonals::OrBringup>,
      StFollowOperator,
      smacc2::SUCCESS>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrBringup,
      client_behaviors::CbKillRx200Camera>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "Entering StKillRx200Camera");
  }
};
}  // namespace carry_my_luggage_phase1
