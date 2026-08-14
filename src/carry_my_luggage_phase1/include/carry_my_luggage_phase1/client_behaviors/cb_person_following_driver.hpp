#pragma once

#include <dirent.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <future>
#include <mutex>
#include <putrabot_interfaces/action/managed_process.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace carry_my_luggage_phase1::client_behaviors
{
inline constexpr const char* kPersonFollowBringupReadyKey = "person_follow_bringup_ready";

class CbPersonFollowingDriver : public smacc2::SmaccAsyncClientBehavior
{
 public:
  CbPersonFollowingDriver() = default;
  ~CbPersonFollowingDriver() override = default;

  void onEntry() override
  {
    using ManagedProcess = putrabot_interfaces::action::ManagedProcess;
    using namespace std::chrono_literals;

    getStateMachine()->setGlobalSMData(kPersonFollowBringupReadyKey, false);
    {
      std::lock_guard<std::mutex> lock(teardown_mutex_);
      teardown_requested_ = false;
    }

    auto node = getNode();
    auto follow_client =
        rclcpp_action::create_client<ManagedProcess>(node, "/bringup/person_follow");

    if (!follow_client->wait_for_action_server(10s))
    {
      RCLCPP_WARN(getLogger(), "/bringup/person_follow action server is not available.");
      postFailureEvent();
      return;
    }

    auto send_goal = [this](const std::shared_ptr<rclcpp_action::Client<ManagedProcess>>& client,
                            const char* action_name) ->
        typename rclcpp_action::ClientGoalHandle<ManagedProcess>::SharedPtr
    {
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
        return nullptr;
      }

      auto goal_handle = goal_future.get();
      if (goal_handle == nullptr)
      {
        RCLCPP_WARN(getLogger(), "%s goal was rejected.", action_name);
        return nullptr;
      }

      RCLCPP_INFO(getLogger(), "%s goal accepted.", action_name);
      return goal_handle;
    };

    auto wait_result =
        [this](
            const std::shared_ptr<rclcpp_action::Client<ManagedProcess>>& client,
            const typename rclcpp_action::ClientGoalHandle<ManagedProcess>::SharedPtr& goal_handle,
            const char* action_name, std::chrono::seconds timeout)
    {
      auto result_future = client->async_get_result(goal_handle);
      const auto start = std::chrono::steady_clock::now();
      while (!isShutdownRequested() && rclcpp::ok() &&
             result_future.wait_for(100ms) != std::future_status::ready)
      {
        if ((std::chrono::steady_clock::now() - start) > timeout)
        {
          RCLCPP_WARN(getLogger(), "%s timed out.", action_name);
          return false;
        }
        rclcpp::sleep_for(100ms);
      }

      if (!rclcpp::ok() || isShutdownRequested())
      {
        return false;
      }

      const auto result = result_future.get();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result != nullptr &&
          result.result->success)
      {
        RCLCPP_INFO(getLogger(), "%s ready: %s", action_name, result.result->message.c_str());
        return true;
      }

      const auto message =
          result.result != nullptr ? result.result->message : std::string("no result message");
      RCLCPP_WARN(getLogger(), "%s failed: %s", action_name, message.c_str());
      return false;
    };
      auto follow_goal_handle = send_goal(follow_client, "/bringup/person_follow");
      if (follow_goal_handle == nullptr)
      {
        postFailureEvent();
        return;
      }
      const bool follow_ready =
          wait_result(follow_client, follow_goal_handle, "/bringup/person_follow", 25s);
      if (follow_ready)
      {
        getStateMachine()->setGlobalSMData(kPersonFollowBringupReadyKey, true);
        postSuccessEvent();
        return;
      };

    getStateMachine()->setGlobalSMData(kPersonFollowBringupReadyKey, false);
    postFailureEvent();
  }

  void requestStopPersonFollowBringup()
  {
    getStateMachine()->setGlobalSMData(kPersonFollowBringupReadyKey, false);

    {
      std::lock_guard<std::mutex> lock(teardown_mutex_);
      if (teardown_requested_)
      {
        RCLCPP_DEBUG(getLogger(),
                     "CbPersonFollowingDriver teardown already "
                     "requested; skipping duplicate.");
        return;
      }
      teardown_requested_ = true;
    }

    const auto logger = getLogger();
    std::thread(
        [logger]()
        {
          const auto stopped_count = stopPersonFollowProcesses();
          if (stopped_count.processes == 0)
          {
            RCLCPP_INFO(logger,
                        "No /bringup/person_follow related processes found "
                        "for SIGINT shutdown.");
            return;
          }

          RCLCPP_WARN(logger, "Stopped %zu /bringup/person_follow launch root process(es).",
                      stopped_count.processes);
        })
        .detach();
  }

 private:
  struct ProcessInfo
  {
    pid_t pid{-1};
    pid_t ppid{-1};
    pid_t pgid{-1};
    std::string cmdline;
  };

  struct StopCounts
  {
    std::size_t processes{0};
  };

  struct StopTargets
  {
    std::size_t roots{0};
    std::vector<pid_t> pids;
    std::vector<pid_t> pgids;
  };

  static bool readCmdline(pid_t pid, std::string& cmdline)
  {
    std::ifstream file("/proc/" + std::to_string(pid) + "/cmdline", std::ios::binary);
    if (!file.good())
    {
      return false;
    }

    std::ostringstream buffer;
    buffer << file.rdbuf();
    cmdline = buffer.str();
    std::replace(cmdline.begin(), cmdline.end(), '\0', ' ');
    while (!cmdline.empty() && cmdline.back() == ' ')
    {
      cmdline.pop_back();
    }
    return !cmdline.empty();
  }

  static bool readProcessIds(pid_t pid, pid_t& ppid, pid_t& pgid)
  {
    std::ifstream file("/proc/" + std::to_string(pid) + "/stat");
    std::string line;
    if (!std::getline(file, line))
    {
      return false;
    }

    const auto right_paren = line.rfind(')');
    if (right_paren == std::string::npos || right_paren + 2 >= line.size())
    {
      return false;
    }

    std::istringstream stream(line.substr(right_paren + 2));
    char state = '\0';
    stream >> state >> ppid >> pgid;
    return !stream.fail();
  }

  static std::vector<ProcessInfo> snapshotProcesses()
  {
    std::vector<ProcessInfo> processes;
    DIR* proc_dir = opendir("/proc");
    if (proc_dir == nullptr)
    {
      return processes;
    }

    while (dirent* entry = readdir(proc_dir))
    {
      char* end = nullptr;
      const long parsed_pid = std::strtol(entry->d_name, &end, 10);
      if (end == entry->d_name || *end != '\0' || parsed_pid <= 0)
      {
        continue;
      }

      ProcessInfo info;
      info.pid = static_cast<pid_t>(parsed_pid);
      if (!readCmdline(info.pid, info.cmdline) || !readProcessIds(info.pid, info.ppid, info.pgid))
      {
        continue;
      }
      processes.push_back(std::move(info));
    }

    closedir(proc_dir);
    return processes;
  }

  static bool contains(const std::string& text, const std::string& needle)
  {
    return text.find(needle) != std::string::npos;
  }

  static bool isPersonFollowBringupRoot(const ProcessInfo& process)
  {
    const auto& cmd = process.cmdline;
    const bool person_follow_stack =
        contains(cmd, "ros2 launch person_following person_follow_nav2.launch.py") ||
        (contains(cmd, "person_following") && contains(cmd, "person_follow_nav2.launch.py")) ||
        contains(cmd, "ros2 launch person_follower_master person_follow.launch.py") ||
        (contains(cmd, "person_follower_master") && contains(cmd, "person_follow.launch.py"));

    const bool person_follow_drivers =
        contains(cmd, "ros2 launch putrabot_bringup drivers_bringup.launch.py") &&
        contains(cmd, "enable_person_following:=true");

    return person_follow_stack || person_follow_drivers;
  }

  static void collectDescendants(
      pid_t root_pid, const std::unordered_map<pid_t, std::vector<pid_t>>& children_by_parent,
      std::vector<pid_t>& descendants)
  {
    const auto children_it = children_by_parent.find(root_pid);
    if (children_it == children_by_parent.end())
    {
      return;
    }

    for (const auto child_pid : children_it->second)
    {
      descendants.push_back(child_pid);
      collectDescendants(child_pid, children_by_parent, descendants);
    }
  }

  static void appendUnique(std::vector<pid_t>& values, pid_t value)
  {
    if (value <= 0 || std::find(values.begin(), values.end(), value) != values.end())
    {
      return;
    }
    values.push_back(value);
  }

  static StopTargets findPersonFollowTargets()
  {
    StopTargets targets;
    const auto processes = snapshotProcesses();
    std::unordered_map<pid_t, std::vector<pid_t>> children_by_parent;
    std::unordered_map<pid_t, pid_t> pgid_by_pid;
    for (const auto& process : processes)
    {
      children_by_parent[process.ppid].push_back(process.pid);
      pgid_by_pid[process.pid] = process.pgid;
    }

    const pid_t self_pid = getpid();
    const pid_t self_pgid = getpgrp();
    for (const auto& process : processes)
    {
      if (!isPersonFollowBringupRoot(process))
      {
        continue;
      }

      ++targets.roots;
      std::vector<pid_t> process_tree{process.pid};
      collectDescendants(process.pid, children_by_parent, process_tree);
      for (const auto pid : process_tree)
      {
        if (pid == self_pid)
        {
          continue;
        }
        appendUnique(targets.pids, pid);

        const auto pgid_it = pgid_by_pid.find(pid);
        if (pgid_it != pgid_by_pid.end() && pgid_it->second != self_pgid)
        {
          appendUnique(targets.pgids, pgid_it->second);
        }
      }
    }

    return targets;
  }

  static std::size_t signalTargets(const StopTargets& targets, int signal_value)
  {
    std::size_t signaled = 0;
    for (const auto pgid : targets.pgids)
    {
      errno = 0;
      if (kill(-pgid, signal_value) == 0 || errno == ESRCH)
      {
        ++signaled;
      }
    }

    for (const auto pid : targets.pids)
    {
      errno = 0;
      if (kill(pid, signal_value) == 0 || errno == ESRCH)
      {
        ++signaled;
      }
    }

    return signaled;
  }

  static bool processExists(pid_t pid)
  {
    errno = 0;
    return kill(pid, 0) == 0 || errno == EPERM;
  }

  static bool processGroupExists(pid_t pgid)
  {
    errno = 0;
    return kill(-pgid, 0) == 0 || errno == EPERM;
  }

  static bool targetsStillRunning(const StopTargets& targets)
  {
    for (const auto pid : targets.pids)
    {
      if (processExists(pid))
      {
        return true;
      }
    }

    for (const auto pgid : targets.pgids)
    {
      if (processGroupExists(pgid))
      {
        return true;
      }
    }

    return false;
  }

  static bool waitForTargetsExit(const StopTargets& targets, std::chrono::milliseconds timeout)
  {
    using namespace std::chrono_literals;
    const auto start = std::chrono::steady_clock::now();
    while ((std::chrono::steady_clock::now() - start) < timeout)
    {
      if (!targetsStillRunning(targets))
      {
        return true;
      }
      rclcpp::sleep_for(100ms);
    }

    return !targetsStillRunning(targets);
  }

  static StopCounts stopPersonFollowProcesses()
  {
    using namespace std::chrono_literals;
    const auto targets = findPersonFollowTargets();
    if (targets.roots == 0)
    {
      return StopCounts{0};
    }

    (void)signalTargets(targets, SIGINT);
    if (waitForTargetsExit(targets, 3s))
    {
      return StopCounts{targets.roots};
    }

    (void)signalTargets(targets, SIGTERM);
    if (waitForTargetsExit(targets, 2s))
    {
      return StopCounts{targets.roots};
    }

    (void)signalTargets(targets, SIGKILL);
    (void)waitForTargetsExit(targets, 1s);
    return StopCounts{targets.roots};
  }

  std::mutex teardown_mutex_;
  bool teardown_requested_{false};
};
}  // namespace carry_my_luggage_phase1::client_behaviors
