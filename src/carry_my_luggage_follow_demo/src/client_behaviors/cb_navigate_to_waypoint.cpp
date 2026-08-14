#include "carry_my_luggage_phase1/client_behaviors/cb_navigate_to_waypoint.hpp"

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <putrabot_interfaces/srv/get_waypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <future>
#include <string>

using namespace std::chrono_literals;

namespace
{
constexpr auto kServiceTimeout = std::chrono::seconds(5);
constexpr auto kResponseTimeout = std::chrono::seconds(10);
constexpr auto kNav2ServerTimeout = std::chrono::seconds(10);
constexpr auto kNav2ResultTimeout = std::chrono::seconds(120);
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
CbNavigateToWaypoint::CbNavigateToWaypoint(std::string waypoint_id)
: waypoint_id_(std::move(waypoint_id))
{
}

CbNavigateToWaypoint::CbNavigateToWaypoint(size_t waypoint_index)
: use_index_(true), waypoint_index_(waypoint_index)
{
}

void CbNavigateToWaypoint::onEntry()
{
  using GetWaypoints = putrabot_interfaces::srv::GetWaypoints;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  auto node = getNode();
  auto get_client = node->create_client<GetWaypoints>("waypoints/get");

  if (use_index_) {
    RCLCPP_WARN(
      getLogger(),
      "Waypoint index (%zu) is not supported by waypoints/get. Provide a waypoint id instead.",
      waypoint_index_);
    postFailureEvent();
    return;
  }

  if (waypoint_id_.empty()) {
    RCLCPP_WARN(getLogger(), "Waypoint id is empty.");
    postFailureEvent();
    return;
  }

  if (!get_client->wait_for_service(kServiceTimeout)) {
    RCLCPP_WARN(getLogger(), "waypoints/get service is not available.");
    postFailureEvent();
    return;
  }

  auto request = std::make_shared<GetWaypoints::Request>();
  request->waypoint_id = waypoint_id_;
  auto future = get_client->async_send_request(request);
  const auto start = std::chrono::steady_clock::now();

  while (!isShutdownRequested() && rclcpp::ok() && future.wait_for(100ms) != std::future_status::ready) {
    if ((std::chrono::steady_clock::now() - start) > kResponseTimeout) {
      RCLCPP_WARN(getLogger(), "Timed out waiting for waypoints/get response.");
      postFailureEvent();
      return;
    }
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postFailureEvent();
    return;
  }

  const auto response = future.get();
  if (response == nullptr || !response->success) {
    const auto message = response != nullptr ? response->message : std::string("null response");
    RCLCPP_WARN(getLogger(), "Failed to get waypoints: %s", message.c_str());
    postFailureEvent();
    return;
  }

  const auto & target_goal = response->goal;
  const auto & target_id = waypoint_id_;

  auto nav_client = rclcpp_action::create_client<NavigateToPose>(node, "/navigate_to_pose");
  if (!nav_client->wait_for_action_server(kNav2ServerTimeout)) {
    RCLCPP_WARN(getLogger(), "/navigate_to_pose action server is not available.");
    postFailureEvent();
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose = target_goal;
  goal.pose.header.stamp = node->now();

  RCLCPP_INFO(
    getLogger(), "Navigating to waypoint id='%s' in frame '%s'.",
    target_id.c_str(), goal.pose.header.frame_id.c_str());

  auto goal_future = nav_client->async_send_goal(goal);
  while (!isShutdownRequested() && rclcpp::ok() && goal_future.wait_for(100ms) != std::future_status::ready) {
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postFailureEvent();
    return;
  }

  auto goal_handle = goal_future.get();
  if (goal_handle == nullptr) {
    RCLCPP_WARN(getLogger(), "/navigate_to_pose goal was rejected.");
    postFailureEvent();
    return;
  }

  auto result_future = nav_client->async_get_result(goal_handle);
  const auto result_start = std::chrono::steady_clock::now();
  while (!isShutdownRequested() && rclcpp::ok() &&
    result_future.wait_for(100ms) != std::future_status::ready)
  {
    if ((std::chrono::steady_clock::now() - result_start) > kNav2ResultTimeout) {
      RCLCPP_WARN(getLogger(), "Timed out waiting for /navigate_to_pose result.");
      postFailureEvent();
      return;
    }
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postFailureEvent();
    return;
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(getLogger(), "/navigate_to_pose succeeded.");
    postSuccessEvent();
    return;
  }

  RCLCPP_WARN(getLogger(), "/navigate_to_pose failed.");
  postFailureEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
