#pragma once

#include <boost/mpl/list.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_breadcrumb.hpp"
#include "carry_my_luggage_phase1/client_behaviors/cb_wait_for_button_press.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_terminal.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"

namespace carry_my_luggage_phase1
{
struct SsNavigateToBag;
struct StAborted;
struct EvWaitForButtonPressReady : boost::statechart::event<EvWaitForButtonPressReady>
{
};

struct StWaitForButtonPress : smacc2::SmaccState<StWaitForButtonPress, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;
  using EvButtonPressSuccess = smacc2::EvCbSuccess<
    client_behaviors::CbWaitForButtonPress,
    orthogonals::OrTerminal>;
  using EvBreadCrumbSuccess = smacc2::EvCbSuccess<
    client_behaviors::CbBreadCrumb,
    orthogonals::OrTerminal>;

  using reactions = boost::mpl::list<
    boost::statechart::custom_reaction<EvButtonPressSuccess>,
    boost::statechart::custom_reaction<EvBreadCrumbSuccess>,
    smacc2::Transition<EvWaitForButtonPressReady, SsNavigateToBag, smacc2::SUCCESS>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbWaitForButtonPress,
        orthogonals::OrTerminal>,
      StAborted,
      smacc2::ABORT>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbBreadCrumb,
        orthogonals::OrTerminal>,
      StAborted,
      smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrTerminal,
      client_behaviors::CbBreadCrumb>();
    configure_orthogonal<
      orthogonals::OrTerminal,
      client_behaviors::CbWaitForButtonPress>();
  }

  void onEntry()
  {
    button_pressed_ = false;
    breadcrumb_ready_ = false;
    RCLCPP_INFO(
      getLogger(),
      "Entering StWaitForButtonPress: waiting for button press and breadcrumb teach start");
  }

  boost::statechart::result react(const EvButtonPressSuccess &)
  {
    button_pressed_ = true;
    RCLCPP_INFO(getLogger(), "Button press detected.");
    maybeAdvance();
    return this->discard_event();
  }

  boost::statechart::result react(const EvBreadCrumbSuccess &)
  {
    breadcrumb_ready_ = true;
    RCLCPP_INFO(getLogger(), "Breadcrumb teach mode started.");
    maybeAdvance();
    return this->discard_event();
  }

private:
  void maybeAdvance()
  {
    if (button_pressed_ && breadcrumb_ready_) {
      this->template postEvent<EvWaitForButtonPressReady>();
    }
  }

  bool button_pressed_{false};
  bool breadcrumb_ready_{false};
};
}  // namespace carry_my_luggage_phase1
