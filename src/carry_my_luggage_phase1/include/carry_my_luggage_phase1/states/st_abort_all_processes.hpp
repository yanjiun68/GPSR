#pragma once

#include <boost/mpl/list.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_abort_all_processes.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_terminal.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"

namespace carry_my_luggage_phase1
{
struct StCompleted;

struct StAbortAllProcesses : smacc2::SmaccState<StAbortAllProcesses, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<
        client_behaviors::CbAbortAllProcesses,
        orthogonals::OrTerminal>,
      StCompleted,
      smacc2::SUCCESS>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbAbortAllProcesses,
        orthogonals::OrTerminal>,
      StAbortAllProcesses,
      smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrTerminal,
      client_behaviors::CbAbortAllProcesses>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "Entering StAbortAllProcesses");
  }
};
}  // namespace carry_my_luggage_phase1
