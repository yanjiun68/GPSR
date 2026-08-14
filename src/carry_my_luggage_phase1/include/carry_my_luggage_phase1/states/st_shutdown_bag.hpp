#pragma once

#include <boost/mpl/list.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_shutdown_bag.hpp"
#include "carry_my_luggage_phase1/events/ev_retry_limit_exceeded.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_bringup.hpp"
#include "carry_my_luggage_phase1/superstates/ss_navigate_to_bag.hpp"

namespace carry_my_luggage_phase1
{
struct StAborted;
struct StExecuteGrasp;

struct StShutdownBag : smacc2::SmaccState<StShutdownBag, SsNavigateToBag>
{
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<
        client_behaviors::CbShutdownBag,
        orthogonals::OrBringup>,
      StExecuteGrasp,
      smacc2::SUCCESS>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbShutdownBag,
        orthogonals::OrBringup>,
      StShutdownBag,
      smacc2::ABORT>,
    smacc2::Transition<
      events::EvRetryLimitExceeded,
      StAborted,
      smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrBringup,
      client_behaviors::CbShutdownBag>();
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "Entering StShutdownBag"); }
};
}  // namespace carry_my_luggage_phase1
