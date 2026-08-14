#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/mpl/list.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_load_waypoint.hpp"
#include "carry_my_luggage_phase1/client_behaviors/cb_start_full_system_bringup.hpp"
#include "carry_my_luggage_phase1/events/ev_bringup_and_waypoints_ready.hpp"
#include "carry_my_luggage_phase1/events/ev_retry_limit_exceeded.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_bringup.hpp"
#include "carry_my_luggage_phase1/orthogonals/or_terminal.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"
#include "carry_my_luggage_phase1/states/st_go_to_destination.hpp"

namespace carry_my_luggage_phase1
{
struct StAborted;
struct StExecuteGrasp;

struct StStartupBringup : smacc2::SmaccState<StStartupBringup, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;
  using EvBringupSuccess = smacc2::EvCbSuccess<
    client_behaviors::CbStartFullSystemBringup,
    orthogonals::OrBringup>;
  using EvWaypointsSuccess = smacc2::EvCbSuccess<
    client_behaviors::CbLoadWaypoint,
    orthogonals::OrTerminal>;

  using reactions = boost::mpl::list<
    smacc2::Transition<
      events::EvBringupAndWaypointsReady,
      StExecuteGrasp,
      smacc2::SUCCESS>,
    boost::statechart::custom_reaction<EvBringupSuccess>,
    boost::statechart::custom_reaction<EvWaypointsSuccess>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbStartFullSystemBringup,
        orthogonals::OrBringup>,
      StStartupBringup,
      smacc2::ABORT>,
    smacc2::Transition<
      smacc2::EvCbFailure<
        client_behaviors::CbLoadWaypoint,
        orthogonals::OrTerminal>,
      StStartupBringup,
      smacc2::ABORT>,
    smacc2::Transition<
      events::EvRetryLimitExceeded,
      StAborted,
      smacc2::ABORT>>;

  static void staticConfigure()
  {
    std::string waypoint_yaml_path;
    try {
      waypoint_yaml_path =
        ament_index_cpp::get_package_share_directory("carry_my_luggage_follow_demo") +
        "/config/waypoints.yaml";
    } catch (const std::exception &) {
      waypoint_yaml_path =
        "/home/yanjiun68/workspace/WS1/PutrabotUPM-project/src/carry_my_luggage_follow_demo/config/waypoints.yaml";
    }

    configure_orthogonal<
      orthogonals::OrBringup,
      client_behaviors::CbStartFullSystemBringup>();
    configure_orthogonal<
      orthogonals::OrTerminal,
      client_behaviors::CbLoadWaypoint>(waypoint_yaml_path);
  }

  void onEntry()
  {
    bringup_ready_ = false;
    waypoints_ready_ = false;
    RCLCPP_INFO(
      getLogger(),
      "Entering StStartupBringup: waiting for full_system_bringup and waypoint load to both succeed");
  }

  boost::statechart::result react(const EvBringupSuccess &)
  {
    bringup_ready_ = true;
    RCLCPP_INFO(getLogger(), "Startup bringup succeeded.");
    maybeTriggerBothReady();
    return this->discard_event();
  }

  boost::statechart::result react(const EvWaypointsSuccess &)
  {
    waypoints_ready_ = true;
    RCLCPP_INFO(getLogger(), "Waypoint preload succeeded.");
    maybeTriggerBothReady();
    return this->discard_event();
  }

private:
  void maybeTriggerBothReady()
  {
    if (bringup_ready_ && waypoints_ready_) {
      postEvent<events::EvBringupAndWaypointsReady>();
    }
  }

  bool bringup_ready_{false};
  bool waypoints_ready_{false};
};
}  // namespace carry_my_luggage_phase1
