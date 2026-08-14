#pragma once

#include <boost/mpl/list.hpp>
#include <smacc2/smacc.hpp>

#include <chrono>

#include "carry_my_luggage_phase1/client_behaviors/cb_abort_stage1_notice.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_terminal.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"
#include "carry_my_luggage_phase1/utils/bringup_shutdown.hpp"

namespace carry_my_luggage_phase1
{
using namespace std::chrono_literals;

struct StCompleted;

struct StAbortStage1Notice
: smacc2::SmaccState<StAbortStage1Notice, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<
        client_behaviors::CbAbortStage1Notice,
        orthogonals::OrTerminal>,
      StCompleted,
      smacc2::SUCCESS>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrTerminal,
      client_behaviors::CbAbortStage1Notice>();
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "Entering StAbortStage1Notice"); }

  void onExit()
  {
    while (rclcpp::ok() && !utils::shutdownHeavyBringupNodes(this)) {
      RCLCPP_ERROR(
        getLogger(),
        "Heavy nodes are still running. Retrying shutdown before exiting StAbortStage1Notice.");
      rclcpp::sleep_for(500ms);
    }
  }
};
}  // namespace carry_my_luggage_phase1
