#include "carry_my_luggage_phase1/client_behaviors/cb_abort_all_processes.hpp"
#include "carry_my_luggage_phase1/utils/bringup_pid_registry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <sys/types.h>

#include <cerrno>

using namespace std::chrono_literals;

namespace carry_my_luggage_phase1::client_behaviors
{
void CbAbortAllProcesses::onEntry()
{
  // Request a graceful stop from the person_follow stack so it drops /person_follow/enable.
  {
    auto node = getNode();
    auto stop_client = node->create_client<std_srvs::srv::Trigger>("/person_follow/stop");
    if (stop_client->wait_for_service(500ms)) {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future = stop_client->async_send_request(req);
      if (rclcpp::spin_until_future_complete(node, future, 1s) == rclcpp::FutureReturnCode::SUCCESS &&
          future.get()->success) {
        RCLCPP_INFO(getLogger(), "/person_follow/stop succeeded: %s", future.get()->message.c_str());
      } else {
        RCLCPP_WARN(getLogger(), "/person_follow/stop failed or timed out.");
      }
    } else {
      RCLCPP_WARN(getLogger(), "/person_follow/stop service not available; skipping graceful stop.");
    }
  }

  auto * state_machine = getStateMachine();
  std::int64_t bringup_pid = -1;
  state_machine->getGlobalSMData("bringup_pid", bringup_pid);

  if (bringup_pid <= 0) {
    utils::clearBringupPid();
    RCLCPP_INFO(getLogger(), "No tracked full_system_bringup process. Cleanup is already complete.");
    postSuccessEvent();
    return;
  }

  const auto process_group_id = static_cast<pid_t>(bringup_pid);

  const auto process_group_running = [](pid_t pgid) {
      if (pgid <= 0) {
        return false;
      }

      const auto ret = kill(-pgid, 0);
      return ret == 0 || errno == EPERM;
    };

  const auto wait_for_exit = [process_group_running](pid_t pgid, std::chrono::milliseconds timeout) {
      const auto start = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - start < timeout) {
        if (!process_group_running(pgid)) {
          return true;
        }
        rclcpp::sleep_for(200ms);
      }
      return !process_group_running(pgid);
    };

  if (!process_group_running(process_group_id)) {
    utils::clearBringupPid();
    state_machine->setGlobalSMData("bringup_pid", static_cast<std::int64_t>(-1));
    RCLCPP_INFO(getLogger(), "full_system_bringup process group already stopped (pid=%ld).", bringup_pid);
    postSuccessEvent();
    return;
  }

  RCLCPP_WARN(
    getLogger(), "Stopping heavy nodes from full_system_bringup process group (pid=%ld).",
    bringup_pid);

  const auto send_signal = [process_group_id](int signal_value) {
      const auto ret = kill(-process_group_id, signal_value);
      return ret == 0 || errno == ESRCH;
    };

  if (send_signal(SIGINT) && wait_for_exit(process_group_id, std::chrono::seconds(5))) {
    utils::clearBringupPid();
    state_machine->setGlobalSMData("bringup_pid", static_cast<std::int64_t>(-1));
    RCLCPP_INFO(getLogger(), "Heavy nodes stopped after SIGINT.");
    postSuccessEvent();
    return;
  }

  if (send_signal(SIGTERM) && wait_for_exit(process_group_id, std::chrono::seconds(2))) {
    utils::clearBringupPid();
    state_machine->setGlobalSMData("bringup_pid", static_cast<std::int64_t>(-1));
    RCLCPP_INFO(getLogger(), "Heavy nodes stopped after SIGTERM.");
    postSuccessEvent();
    return;
  }

  if (send_signal(SIGKILL) && wait_for_exit(process_group_id, std::chrono::seconds(1))) {
    utils::clearBringupPid();
    state_machine->setGlobalSMData("bringup_pid", static_cast<std::int64_t>(-1));
    RCLCPP_INFO(getLogger(), "Heavy nodes stopped after SIGKILL.");
    postSuccessEvent();
    return;
  }

  RCLCPP_ERROR(getLogger(), "Heavy node process group %d is still running.", process_group_id);
  postFailureEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
