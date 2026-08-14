#pragma once

#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_ready_pose.hpp"
#include "carry_my_luggage_phase1/client_behaviors/cb_select_bag.hpp"
#include "carry_my_luggage_phase1/events/ev_retry_limit_exceeded.hpp"
#include "carry_my_luggage_phase1/events/ev_select_bag_ready.hpp"
#include "carry_my_luggage_phase1/states/st_bringup_bag.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_bringup.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_manipulation.hpp"
#include "carry_my_luggage_phase1/superstates/ss_navigate_to_bag.hpp"

namespace carry_my_luggage_phase1
{
struct StAborted;
struct StShutdownBag;

struct StSelectBag : smacc2::SmaccState<StSelectBag, SsNavigateToBag>
{
  using SmaccState::SmaccState;
  using EvSelectBagSuccess = smacc2::EvCbSuccess<
    client_behaviors::CbSelectBag,
    orthogonals::OrManipulation>;
  using EvReadyPoseSuccess = smacc2::EvCbSuccess<
    client_behaviors::CbReadyPose,
    orthogonals::OrBringup>;

  using reactions = boost::mpl::list<
    smacc2::Transition<
      events::EvSelectBagReady,
      StShutdownBag,
      smacc2::SUCCESS>,
    boost::statechart::custom_reaction<EvSelectBagSuccess>,
    boost::statechart::custom_reaction<EvReadyPoseSuccess>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbSelectBag,
        orthogonals::OrManipulation>,
      StBringupBag,
      smacc2::ABORT>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbReadyPose,
        orthogonals::OrBringup>,
      StSelectBag,
      smacc2::ABORT>,
    smacc2::Transition<
      events::EvRetryLimitExceeded,
      StAborted,
      smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<
      orthogonals::OrBringup,
      client_behaviors::CbReadyPose>();
    configure_orthogonal<
      orthogonals::OrManipulation,
      client_behaviors::CbSelectBag>();
  }

  void onEntry()
  {
    select_bag_ready_ = false;
    ready_pose_ready_ = false;
    RCLCPP_INFO(getLogger(), "Entering StSelectBag: waiting for CbSelectBag and CbReadyPose");
  }

  boost::statechart::result react(const EvSelectBagSuccess &)
  {
    select_bag_ready_ = true;
    RCLCPP_INFO(getLogger(), "CbSelectBag succeeded.");
    maybeTriggerBothReady();
    return this->discard_event();
  }

  boost::statechart::result react(const EvReadyPoseSuccess &)
  {
    ready_pose_ready_ = true;
    RCLCPP_INFO(getLogger(), "CbReadyPose succeeded.");
    maybeTriggerBothReady();
    return this->discard_event();
  }

private:
  void maybeTriggerBothReady()
  {
    if (select_bag_ready_ && ready_pose_ready_) {
      postEvent<events::EvSelectBagReady>();
    }
  }

  bool select_bag_ready_{false};
  bool ready_pose_ready_{false};
};
}  // namespace carry_my_luggage_phase1
