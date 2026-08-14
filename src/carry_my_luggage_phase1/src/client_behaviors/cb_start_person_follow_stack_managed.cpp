#include "carry_my_luggage_phase1/client_behaviors/cb_start_person_follow_stack_managed.hpp"

#include <putrabot_interfaces/action/managed_process.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <future>

using namespace std::chrono_literals;

namespace
{
constexpr auto kServerTimeout = std::chrono::seconds(10);
constexpr auto kBringupResultTimeout = std::chrono::seconds(20);
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbStartPersonFollowStackManaged::onEntry()
{
  using ManagedProcess = putrabot_interfaces::action::ManagedProcess;

  auto node = getNode();
  auto client = rclcpp_action::create_client<ManagedProcess>(node, "/bringup/person_follow");

  if (!client->wait_for_action_server(kServerTimeout))
  {
    RCLCPP_WARN(getLogger(), "/bringup/person_follow action server is not available.");
    postFailureEvent();
    return;
  }

  ManagedProcess::Goal goal;
  goal.force_restart = false;

  auto goal_future = client->async_send_goal(goal);
  while (!isShutdownRequested() && rclcpp::ok() &&
         goal_future.wait_for(100ms) != std::future_status::ready)
  {
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested())
  {
    postFailureEvent();
    return;
  }

  auto goal_handle = goal_future.get();
  if (goal_handle == nullptr)
  {
    RCLCPP_WARN(getLogger(), "/bringup/person_follow goal was rejected.");
    postFailureEvent();
    return;
  }

  auto result_future = client->async_get_result(goal_handle);
  const auto start = std::chrono::steady_clock::now();
  while (!isShutdownRequested() && rclcpp::ok() &&
         result_future.wait_for(100ms) != std::future_status::ready)
  {
    if ((std::chrono::steady_clock::now() - start) > kBringupResultTimeout)
    {
      RCLCPP_WARN(getLogger(), "/bringup/person_follow timed out.");
      postFailureEvent();
      return;
    }
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested())
  {
    postFailureEvent();
    return;
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success)
  {
    RCLCPP_INFO(getLogger(), "/bringup/person_follow ready: %s", result.result->message.c_str());
    postSuccessEvent();
    return;
  }

  RCLCPP_WARN(getLogger(), "/bringup/person_follow failed.");
  postFailureEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
