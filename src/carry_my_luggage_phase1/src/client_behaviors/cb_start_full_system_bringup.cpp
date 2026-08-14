#include "carry_my_luggage_phase1/client_behaviors/cb_start_full_system_bringup.hpp"

#include "carry_my_luggage_phase1/events/ev_retry_limit_exceeded.hpp"
#include "carry_my_luggage_phase1/retry_policy.hpp"
#include "carry_my_luggage_phase1/utils/bringup_pid_registry.hpp"

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cstdint>
#include <future>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace
{
constexpr const char * kBringupCommand =
  "ros2 launch putrabot_bringup carry_my_luggage_bringup.launch.py enable_bag_follow:=false";

constexpr auto kBringupReadyTimeout = std::chrono::seconds(90);
constexpr auto kRetryDelay = std::chrono::seconds(2);
bool isProcessRunning(pid_t pid)
{
  if (pid <= 0) {
    return false;
  }

  int status = 0;
  const auto ret = waitpid(pid, &status, WNOHANG);
  return ret == 0;
}

pid_t launchDetachedProcess(const std::string & command)
{
  const auto pid = fork();
  if (pid < 0) {
    return -1;
  }

  if (pid == 0) {
    setsid();
    execl("/bin/bash", "bash", "-lc", command.c_str(), static_cast<char *>(nullptr));
    _exit(127);
  }

  return pid;
}

bool waitForControllerManager(
  const rclcpp::Node::SharedPtr & node, std::chrono::seconds timeout)
{
  auto client =
    node->create_client<controller_manager_msgs::srv::ListControllers>(
    "/rx200/controller_manager/list_controllers");

  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && (std::chrono::steady_clock::now() - start) < timeout) {
    if (client->wait_for_service(500ms)) {
      return true;
    }
  }

  return false;
}

bool waitForReadyService(
  const rclcpp::Node::SharedPtr & node, std::chrono::seconds timeout)
{
  auto client = node->create_client<std_srvs::srv::Trigger>("/grasp/execute_ready");
  const auto start = std::chrono::steady_clock::now();

  while (rclcpp::ok() && (std::chrono::steady_clock::now() - start) < timeout) {
    if (client->wait_for_service(500ms)) {
      return true;
    }
  }

  return false;
}

// bool callGraspReadyCheck(
//   const rclcpp::Node::SharedPtr & node, std::string & response_message)
// {
//   auto client = node->create_client<std_srvs::srv::Trigger>("/grasp/execute_ready");

//   if (!client->wait_for_service(500ms)) {
//     response_message = "/grasp/execute_ready service not available";
//     return false;
//   }

//   auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//   auto future = client->async_send_request(request);

//   const auto start = std::chrono::steady_clock::now();
//   while (rclcpp::ok() && future.wait_for(100ms) != std::future_status::ready) {
//     if ((std::chrono::steady_clock::now() - start) > kReadyResponseTimeout) {
//       response_message = "Timed out waiting for /grasp/execute_ready response";
//       return false;
//     }
//   }

//   if (!rclcpp::ok()) {
//     response_message = "ROS shutdown while waiting for /grasp/execute_ready";
//     return false;
//   }

//   const auto response = future.get();
//   if (response == nullptr) {
//     response_message = "Null response from /grasp/execute_ready";
//     return false;
//   }

//   response_message = response->message;
//   return response->success;
// }
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbStartFullSystemBringup::onEntry()
{
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

  std::int64_t bringup_pid = -1;
  getStateMachine()->getGlobalSMData("bringup_pid", bringup_pid);

  if (!isProcessRunning(static_cast<pid_t>(bringup_pid))) {
    bringup_pid = static_cast<std::int64_t>(launchDetachedProcess(kBringupCommand));
    if (bringup_pid <= 0) {
      RCLCPP_ERROR(getLogger(), "Failed to spawn full_system_bringup process.");
      rclcpp::sleep_for(kRetryDelay);
      postRetryOutcome();
      return;
    }

    getStateMachine()->setGlobalSMData("bringup_pid", bringup_pid);
    utils::setBringupPid(bringup_pid);
    RCLCPP_INFO(getLogger(), "Started full_system_bringup with pid=%ld", bringup_pid);
  } else {
    utils::setBringupPid(bringup_pid);
    RCLCPP_INFO(getLogger(), "Reusing full_system_bringup pid=%ld", bringup_pid);
  }

  const auto node = getNode();
  const auto start = std::chrono::steady_clock::now();

  while (!isShutdownRequested() && rclcpp::ok()) {
    if (!isProcessRunning(static_cast<pid_t>(bringup_pid))) {
      RCLCPP_ERROR(getLogger(), "full_system_bringup exited before grasp ready was detected.");
      rclcpp::sleep_for(kRetryDelay);
      postRetryOutcome();
      return;
    }

    const bool controller_ready = waitForControllerManager(node, 1s);
    const bool ready_service_available = waitForReadyService(node, 1s);

    if (controller_ready && ready_service_available) {
      getStateMachine()->setGlobalSMData("retry_count", 0);
      RCLCPP_INFO(
        getLogger(),
        "Startup dependencies are available: controller_manager and /grasp/execute_ready.");
      postSuccessEvent();
      return;
    }

    RCLCPP_INFO_THROTTLE(
      getLogger(), *node->get_clock(), 2000,
      "Waiting for startup dependencies: controller_manager=%d /grasp/execute_ready=%d",
      controller_ready, ready_service_available);

    if ((std::chrono::steady_clock::now() - start) > kBringupReadyTimeout) {
      RCLCPP_ERROR(getLogger(), "Timed out waiting for startup dependencies.");
      rclcpp::sleep_for(kRetryDelay);
      postRetryOutcome();
      return;
    }
    rclcpp::sleep_for(500ms);
  }

  postRetryOutcome();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
