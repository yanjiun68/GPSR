#pragma once

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace sm_receptionist::task_receptionist::process_utils
{
// Section: registry paths shared by the lifecycle wrapper and the SMACC2 child.
inline const std::filesystem::path kRegistryDir{"/tmp/sm_receptionist_task_receptionist"};
inline const std::filesystem::path kInsightFacePgidFile{kRegistryDir / "active_insightface_pgid"};

// Special helper: reap a direct child process without blocking the caller.
inline bool reapChildIfExited(pid_t pid)
{
  if (pid <= 0) {
    return false;
  }

  int status = 0;
  while (true) {
    const auto ret = waitpid(pid, &status, WNOHANG);
    if (ret == pid) {
      return true;
    }
    if (ret < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
}

// Special helper: check whether any process still belongs to a process group.
inline bool processGroupRunning(pid_t process_group_id)
{
  if (process_group_id <= 0) {
    return false;
  }

  reapChildIfExited(process_group_id);
  const auto ret = kill(-process_group_id, 0);
  return ret == 0 || errno == EPERM;
}

// Special helper: quote one shell token for a detached bash command.
inline std::string shellQuote(const std::string & value)
{
  std::string quoted = "'";
  for (const char ch : value) {
    if (ch == '\'') {
      quoted += "'\\''";
    } else {
      quoted += ch;
    }
  }
  quoted += "'";
  return quoted;
}

// Special helper: launch a detached process group for a ROS launch or executable command.
inline pid_t launchDetachedProcess(const std::string & command)
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

// Special helper: write a process group id to a small registry file.
inline void writePgid(const std::filesystem::path & path, pid_t process_group_id)
{
  std::error_code ec;
  std::filesystem::create_directories(kRegistryDir, ec);
  std::ofstream out(path);
  if (out.good()) {
    out << static_cast<std::int64_t>(process_group_id) << '\n';
  }
}

// Special helper: read a process group id from a small registry file.
inline pid_t readPgid(const std::filesystem::path & path)
{
  std::ifstream in(path);
  std::int64_t pid = -1;
  if (in.good()) {
    in >> pid;
  }
  return static_cast<pid_t>(pid);
}

// Special helper: remove a process group registry file.
inline void clearPgid(const std::filesystem::path & path)
{
  std::error_code ec;
  std::filesystem::remove(path, ec);
}

// Special helper: save the active InsightFace process group for cross-process cleanup.
inline void writeInsightFacePgid(pid_t process_group_id)
{
  writePgid(kInsightFacePgidFile, process_group_id);
}

// Special helper: read the active InsightFace process group if one exists.
inline pid_t readInsightFacePgid()
{
  return readPgid(kInsightFacePgidFile);
}

// Special helper: clear the active InsightFace process-group registry.
inline void clearInsightFacePgid()
{
  clearPgid(kInsightFacePgidFile);
}

// Special helper: invoke pkill against a process group with a named signal.
inline int runPkillForGroup(pid_t process_group_id, const std::string & signal_name)
{
  const auto pid = fork();
  if (pid < 0) {
    return -1;
  }

  if (pid == 0) {
    const std::string signal_arg = "-" + signal_name;
    const std::string group_arg = std::to_string(static_cast<int>(process_group_id));
    execlp(
      "pkill",
      "pkill",
      signal_arg.c_str(),
      "-g",
      group_arg.c_str(),
      static_cast<char *>(nullptr));
    _exit(127);
  }

  int status = 0;
  while (waitpid(pid, &status, 0) < 0) {
    if (errno != EINTR) {
      return -1;
    }
  }

  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  return -1;
}

// Special helper: wait for a process group to exit within a bounded timeout.
inline bool waitForGroupExit(pid_t process_group_id, std::chrono::milliseconds timeout)
{
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout) {
    if (!processGroupRunning(process_group_id)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return !processGroupRunning(process_group_id);
}

// Special helper: stop a process group with INT, TERM, then KILL escalation.
inline bool stopProcessGroupWithPkill(
  pid_t process_group_id,
  const std::string & label,
  const rclcpp::Logger & logger)
{
  using namespace std::chrono_literals;

  if (process_group_id <= 0) {
    return true;
  }

  if (!processGroupRunning(process_group_id)) {
    RCLCPP_INFO(
      logger,
      "%s process group %d is already stopped.",
      label.c_str(),
      static_cast<int>(process_group_id));
    return true;
  }

  const struct StopStep
  {
    const char * signal_name;
    std::chrono::milliseconds wait_time;
  } steps[] = {
    {"INT", 3000ms},
    {"TERM", 2000ms},
    {"KILL", 1000ms},
  };

  for (const auto & step : steps) {
    RCLCPP_INFO(
      logger,
      "Stopping %s process group %d with pkill -%s -g %d.",
      label.c_str(),
      static_cast<int>(process_group_id),
      step.signal_name,
      static_cast<int>(process_group_id));

    const int pkill_status = runPkillForGroup(process_group_id, step.signal_name);
    if (pkill_status != 0 && pkill_status != 1) {
      RCLCPP_WARN(
        logger,
        "pkill -%s -g %d returned status %d.",
        step.signal_name,
        static_cast<int>(process_group_id),
        pkill_status);
    }

    if (waitForGroupExit(process_group_id, step.wait_time)) {
      RCLCPP_INFO(
        logger,
        "%s process group %d stopped.",
        label.c_str(),
        static_cast<int>(process_group_id));
      return true;
    }
  }

  RCLCPP_ERROR(
    logger,
    "%s process group %d is still running after pkill escalation.",
    label.c_str(),
    static_cast<int>(process_group_id));
  return false;
}

// Special helper: stop the registered InsightFace launch process if one exists.
inline bool stopRegisteredInsightFace(const rclcpp::Logger & logger)
{
  const auto pgid = readInsightFacePgid();
  const bool stopped = stopProcessGroupWithPkill(pgid, "InsightFace", logger);
  if (stopped) {
    clearInsightFacePgid();
  }
  return stopped;
}
}  // namespace sm_receptionist::task_receptionist::process_utils
