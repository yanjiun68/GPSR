#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"
#include "carry_my_luggage_phase1/states/st_abort_all_processes.hpp"
#include "carry_my_luggage_phase1/states/st_aborted.hpp"
#include "carry_my_luggage_phase1/states/st_ask_pass_bag_confirmation.hpp"
#include "carry_my_luggage_phase1/states/st_completed.hpp"
#include "carry_my_luggage_phase1/states/st_ensure_person_follow_stack.hpp"
#include "carry_my_luggage_phase1/states/st_execute_grasp.hpp"
#include "carry_my_luggage_phase1/states/st_follow_operator.hpp"
#include "carry_my_luggage_phase1/states/st_arm_rest.hpp"
#include "carry_my_luggage_phase1/states/st_ready_follow.hpp"
#include "carry_my_luggage_phase1/states/st_return_home.hpp"
#include "carry_my_luggage_phase1/states/st_startup_bringup.hpp"
#include "carry_my_luggage_phase1/states/st_wait_before_drop.hpp"
#include "carry_my_luggage_phase1/utils/bringup_pid_registry.hpp"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cerrno>
#include <sys/types.h>

namespace
{
using namespace std::chrono_literals;

bool processGroupRunning(pid_t process_group_id)
{
  if (process_group_id <= 0) {
    return false;
  }

  const auto ret = kill(-process_group_id, 0);
  return ret == 0 || errno == EPERM;
}

bool waitForExit(pid_t process_group_id, std::chrono::milliseconds timeout)
{
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout) {
    if (!processGroupRunning(process_group_id)) {
      return true;
    }
    rclcpp::sleep_for(200ms);
  }

  return !processGroupRunning(process_group_id);
}

void forceStopBringupOnShutdown()
{
  const auto bringup_pid = carry_my_luggage_phase1::utils::getBringupPid();
  if (bringup_pid <= 0) {
    return;
  }

  const auto process_group_id = static_cast<pid_t>(bringup_pid);
  const auto send_signal = [process_group_id](int signal_value) {
      const auto ret = kill(-process_group_id, signal_value);
      return ret == 0 || errno == ESRCH;
    };

  if (!processGroupRunning(process_group_id)) {
    carry_my_luggage_phase1::utils::clearBringupPid();
    return;
  }

  if (send_signal(SIGINT) && waitForExit(process_group_id, 3s)) {
    carry_my_luggage_phase1::utils::clearBringupPid();
    return;
  }

  if (send_signal(SIGTERM) && waitForExit(process_group_id, 2s)) {
    carry_my_luggage_phase1::utils::clearBringupPid();
    return;
  }

  if (send_signal(SIGKILL) && waitForExit(process_group_id, 1s)) {
    carry_my_luggage_phase1::utils::clearBringupPid();
    return;
  }

  std::fprintf(
    stderr,
    "[carry_my_luggage_phase1] bringup process group %d is still running after shutdown escalation.\n",
    process_group_id);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::on_shutdown(forceStopBringupOnShutdown);
  smacc2::run<carry_my_luggage_phase1::SmCarryMyLuggagePhase1>();
  rclcpp::shutdown();
  return 0;
}
