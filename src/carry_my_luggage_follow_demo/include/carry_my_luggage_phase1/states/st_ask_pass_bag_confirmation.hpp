#pragma once

#include <boost/mpl/list.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_ask_pass_bag_dialog.hpp"
#include "carry_my_luggage_phase1/events/ev_pass_bag_no.hpp"
#include "carry_my_luggage_phase1/events/ev_pass_bag_yes.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_voice.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"

namespace carry_my_luggage_phase1
{
struct StAborted;
struct StFollowOperator;
struct StWaitBeforeDrop;

struct StAskPassBagConfirmation : smacc2::SmaccState<StAskPassBagConfirmation, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    smacc2::Transition<events::EvPassBagYes, StWaitBeforeDrop, smacc2::SUCCESS>,
    smacc2::Transition<events::EvPassBagNo, StFollowOperator, smacc2::ABORT>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbAskPassBagDialog,
        orthogonals::OrVoice>,
      StAborted,
      smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrVoice,
      client_behaviors::CbAskPassBagDialog>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "Entering StAskPassBagConfirmation");
  }
};
}  // namespace carry_my_luggage_phase1
