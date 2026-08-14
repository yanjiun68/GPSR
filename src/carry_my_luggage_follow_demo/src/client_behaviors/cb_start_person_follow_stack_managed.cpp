#include "carry_my_luggage_phase1/client_behaviors/cb_start_person_follow_stack_managed.hpp"

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <putrabot_interfaces/action/managed_process.hpp>
#include <putrabot_interfaces/srv/get_person_follow_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <future>

using namespace std::chrono_literals;

namespace
{
constexpr auto kServerTimeout = std::chrono::seconds(10);
constexpr auto kBringupResultTimeout = std::chrono::seconds(20);
constexpr auto kFollowStatusTimeout = std::chrono::seconds(10);
constexpr auto kNav2ServerTimeout = std::chrono::seconds(15);

bool waitForPersonFollowStatusService(
  const rclcpp::Node::SharedPtr & node, const rclcpp::Logger & logger,
  const std::function<bool()> & shutdown_requested)
{
  using GetPersonFollowStatus = putrabot_interfaces::srv::GetPersonFollowStatus;

  auto client = node->create_client<GetPersonFollowStatus>("/person_follow/get_status");
  const auto start = std::chrono::steady_clock::now();

  while (rclcpp::ok() && !shutdown_requested()) {
    if ((std::chrono::steady_clock::now() - start) > kFollowStatusTimeout) {
      RCLCPP_WARN(logger, "/person_follow/get_status service timed out.");
      return false;
    }

    if (!client->wait_for_service(500ms)) {
      continue;
    }

    auto request = std::make_shared<GetPersonFollowStatus::Request>();
    auto response_future = client->async_send_request(request);

    while (rclcpp::ok() && !shutdown_requested() &&
      response_future.wait_for(100ms) != std::future_status::ready)
    {
      if ((std::chrono::steady_clock::now() - start) > kFollowStatusTimeout) {
        RCLCPP_WARN(logger, "/person_follow/get_status response timed out.");
        return false;
      }
      rclcpp::sleep_for(100ms);
    }

    if (!rclcpp::ok() || shutdown_requested()) {
      return false;
    }

    const auto response = response_future.get();
    RCLCPP_INFO(
      logger,
      "/person_follow/get_status responded: active=%d state=%s safety_state=%s stop_reason=%s",
      response->active, response->state.c_str(), response->safety_state.c_str(),
      response->stop_reason.c_str());
    return true;
  }

  return false;
}

bool waitForNavigateToPoseServer(
  const rclcpp::Node::SharedPtr & node, const rclcpp::Logger & logger,
  const std::function<bool()> & shutdown_requested)
{
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  auto client = rclcpp_action::create_client<NavigateToPose>(node, "/navigate_to_pose");
  const auto start = std::chrono::steady_clock::now();

  while (rclcpp::ok() && !shutdown_requested()) {
    if (client->wait_for_action_server(500ms)) {
      RCLCPP_INFO(logger, "/navigate_to_pose action server is available.");
      return true;
    }

    if ((std::chrono::steady_clock::now() - start) > kNav2ServerTimeout) {
      RCLCPP_WARN(logger, "/navigate_to_pose action server timed out.");
      return false;
    }
  }

  return false;
}
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbStartPersonFollowStackManaged::onEntry()
{
  using ManagedProcess = putrabot_interfaces::action::ManagedProcess;

  auto node = getNode();
  auto client = rclcpp_action::create_client<ManagedProcess>(node, "/bringup/person_follow");

  if (!client->wait_for_action_server(kServerTimeout)) {
    RCLCPP_WARN(getLogger(), "/bringup/person_follow action server is not available.");
    postFailureEvent();
    return;
  }

  ManagedProcess::Goal goal;
  goal.force_restart = false;

  auto goal_future = client->async_send_goal(goal);
  while (!isShutdownRequested() && rclcpp::ok() && goal_future.wait_for(100ms) != std::future_status::ready) {
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postFailureEvent();
    return;
  }

  auto goal_handle = goal_future.get();
  if (goal_handle == nullptr) {
    RCLCPP_WARN(getLogger(), "/bringup/person_follow goal was rejected.");
    postFailureEvent();
    return;
  }

  auto result_future = client->async_get_result(goal_handle);
  const auto start = std::chrono::steady_clock::now();
  while (!isShutdownRequested() && rclcpp::ok() &&
    result_future.wait_for(100ms) != std::future_status::ready)
  {
    if ((std::chrono::steady_clock::now() - start) > kBringupResultTimeout) {
      RCLCPP_WARN(getLogger(), "/bringup/person_follow timed out.");
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
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
    RCLCPP_INFO(getLogger(), "/bringup/person_follow ready: %s", result.result->message.c_str());

    const auto shutdown_requested = [this]() {return this->isShutdownRequested();};

    if (!waitForPersonFollowStatusService(node, getLogger(), shutdown_requested)) {
      postFailureEvent();
      return;
    }

    if (!waitForNavigateToPoseServer(node, getLogger(), shutdown_requested)) {
      postFailureEvent();
      return;
    }

    postSuccessEvent();
    return;
  }

  RCLCPP_WARN(getLogger(), "/bringup/person_follow failed.");
  postFailureEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
