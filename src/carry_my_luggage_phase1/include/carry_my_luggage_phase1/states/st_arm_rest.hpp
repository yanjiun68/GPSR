#pragma once

#include <boost/mpl/list.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_arm_rest.hpp"
#include "carry_my_luggage_phase1/events/ev_retry_limit_exceeded.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_manipulation.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"

namespace carry_my_luggage_phase1
{
struct StAborted;
struct StReturnHome;

struct StArmRest : smacc2::SmaccState<StArmRest, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<
        client_behaviors::CbArmRest,
        orthogonals::OrManipulation>,
      StReturnHome,
      smacc2::SUCCESS>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbArmRest,
        orthogonals::OrManipulation>,
      StArmRest,
      smacc2::ABORT>,
    smacc2::Transition<
      events::EvRetryLimitExceeded,
      StAborted,
      smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrManipulation,
      client_behaviors::CbArmRest>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "Entering StArmRest");
  }
};
}  // namespace carry_my_luggage_phase1
