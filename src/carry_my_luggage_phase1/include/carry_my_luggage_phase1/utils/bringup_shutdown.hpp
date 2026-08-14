#pragma once

#include <rclcpp/rclcpp.hpp>

#include "carry_my_luggage_phase1/utils/bringup_pid_registry.hpp"

#include <chrono>
#include <csignal>
#include <cstdint>
#include <sys/types.h>

#include <cerrno>

namespace carry_my_luggage_phase1::utils
{
namespace detail
{
inline bool processGroupRunning(pid_t process_group_id)
{
  if (process_group_id <= 0) {
    return false;
  }

  const auto result = kill(-process_group_id, 0);
  if (result == 0) {
    return true;  
  }

  return errno == EPERM;
}

inline bool waitForProcessGroupExit(pid_t process_group_id, std::chrono::milliseconds timeout)
{
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout) {
    if (!processGroupRunning(process_group_id)) {
      return true;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  return !processGroupRunning(process_group_id);
}
}  // namespace detail

template<typename StateT>
inline bool shutdownHeavyBringupNodes(StateT * state)
{
  std::int64_t bringup_pid = -1;
  state->getStateMachine().getGlobalSMData("bringup_pid", bringup_pid);

  if (bringup_pid <= 0) {
    clearBringupPid();
    RCLCPP_INFO(
      state->getLogger(),
      "No tracked full_system_bringup process. Heavy nodes are assumed stopped.");
    return true;
  }

  const auto process_group_id = static_cast<pid_t>(bringup_pid);
  if (!detail::processGroupRunning(process_group_id)) {
    RCLCPP_INFO(
      state->getLogger(), "full_system_bringup process group already stopped (pid=%ld).",
      bringup_pid);
    clearBringupPid();
    state->getStateMachine().setGlobalSMData("bringup_pid", static_cast<std::int64_t>(-1));
    return true;
  }

  RCLCPP_WARN(
    state->getLogger(), "Stopping heavy nodes from full_system_bringup process group (pid=%ld).",
    bringup_pid);

  const auto send_signal = [process_group_id](int signal_value) {
      const auto ret = kill(-process_group_id, signal_value);
      return ret == 0 || errno == ESRCH;
    };

  if (!send_signal(SIGINT)) {
    RCLCPP_ERROR(state->getLogger(), "Failed to send SIGINT to process group %d", process_group_id);
  }
  if (detail::waitForProcessGroupExit(process_group_id, std::chrono::seconds(5))) {
    clearBringupPid();
    state->getStateMachine().setGlobalSMData("bringup_pid", static_cast<std::int64_t>(-1));
    RCLCPP_INFO(state->getLogger(), "Heavy nodes stopped after SIGINT.");
    return true;
  }

  if (!send_signal(SIGTERM)) {
    RCLCPP_ERROR(
      state->getLogger(), "Failed to send SIGTERM to process group %d", process_group_id);
  }
  if (detail::waitForProcessGroupExit(process_group_id, std::chrono::seconds(2))) {
    clearBringupPid();
    state->getStateMachine().setGlobalSMData("bringup_pid", static_cast<std::int64_t>(-1));
    RCLCPP_INFO(state->getLogger(), "Heavy nodes stopped after SIGTERM.");
    return true;
  }

  if (!send_signal(SIGKILL)) {
    RCLCPP_ERROR(
      state->getLogger(), "Failed to send SIGKILL to process group %d", process_group_id);
  }
  if (detail::waitForProcessGroupExit(process_group_id, std::chrono::seconds(1))) {
    clearBringupPid();
    state->getStateMachine().setGlobalSMData("bringup_pid", static_cast<std::int64_t>(-1));
    RCLCPP_INFO(state->getLogger(), "Heavy nodes stopped after SIGKILL.");
    return true;
  }

  RCLCPP_ERROR(
    state->getLogger(), "Heavy node process group %d is still running.", process_group_id);
  return false;
}
}  // namespace carry_my_luggage_phase1::utils
