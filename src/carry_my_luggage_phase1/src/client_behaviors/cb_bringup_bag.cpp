#include "carry_my_luggage_phase1/client_behaviors/cb_bringup_bag.hpp"

#include "carry_my_luggage_phase1/events/ev_retry_limit_exceeded.hpp"
#include "carry_my_luggage_phase1/retry_policy.hpp"

#include <putrabot_interfaces/action/managed_process.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <future>

using namespace std::chrono_literals;

namespace
{
constexpr char kBagBringupActionName[] = "/bringup/bag_point_selection";
constexpr auto kServerTimeout = std::chrono::seconds(10);
constexpr auto kBringupResultTimeout = std::chrono::seconds(30);
constexpr auto kRetryDelay = std::chrono::seconds(2);
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbBringupBag::onEntry()
{
  using ManagedProcess = putrabot_interfaces::action::ManagedProcess;

  auto postRetryOutcome = [this]()
    {
      int retry_count = 0;
      getStateMachine()->getGlobalSMData("retry_count", retry_count);
      retry_count++;
      getStateMachine()->setGlobalSMData("retry_count", retry_count);

      if (retry_count > kMaxRetryAttempts) {
        postEvent<events::EvRetryLimitExceeded>();
      } else {
        postFailureEvent();
      }
    };

  auto node = getNode();
  auto client = rclcpp_action::create_client<ManagedProcess>(node, kBagBringupActionName);

  if (!client->wait_for_action_server(kServerTimeout)) {
    RCLCPP_WARN(getLogger(), "%s action server is not available.", kBagBringupActionName);
    rclcpp::sleep_for(kRetryDelay);
    postRetryOutcome();
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

  if (!rclcpp::ok() || isShutdownRequested()) {
    postRetryOutcome();
    return;
  }

  auto goal_handle = goal_future.get();
  if (goal_handle == nullptr) {
    RCLCPP_WARN(getLogger(), "%s goal was rejected.", kBagBringupActionName);
    rclcpp::sleep_for(kRetryDelay);
    postRetryOutcome();
    return;
  }

  auto result_future = client->async_get_result(goal_handle);
  const auto start = std::chrono::steady_clock::now();
  while (!isShutdownRequested() && rclcpp::ok() &&
    result_future.wait_for(100ms) != std::future_status::ready)
  {
    if ((std::chrono::steady_clock::now() - start) > kBringupResultTimeout) {
      RCLCPP_WARN(getLogger(), "%s timed out.", kBagBringupActionName);
      rclcpp::sleep_for(kRetryDelay);
      postRetryOutcome();
      return;
    }
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postRetryOutcome();
    return;
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result != nullptr &&
    result.result->success)
  {
    RCLCPP_INFO(getLogger(), "%s ready: %s", kBagBringupActionName, result.result->message.c_str());
    getStateMachine()->setGlobalSMData("retry_count", 0);
    postSuccessEvent();
    return;
  }

  const auto message = result.result != nullptr ? result.result->message : std::string("null result");
  RCLCPP_WARN(getLogger(), "%s failed: %s", kBagBringupActionName, message.c_str());
  rclcpp::sleep_for(kRetryDelay);
  postRetryOutcome();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
