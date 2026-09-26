// Section: Lifecycle registry manager for preparing, cleaning, and inspecting GPSR
// resources.
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <fstream>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_ai_interfaces/srv/get_lifecycle_skill_state.hpp>
#include <robot_ai_interfaces/srv/manage_lifecycle_skill.hpp>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "robot_yasmin_tasks/runtime.hpp"

namespace robot_yasmin_tasks {

// Section: Registry data and lifecycle service orchestration for managed GPSR
// dependencies.
struct SkillLifecycleSpec {
  std::string name;
  std::string conflict_group;
  std::vector<std::string> lifecycle_nodes;
  std::string execution_interface;
  std::string interface_type;
  std::string implementation{"real"};
  bool production_enabled{true};
};

class GpsrLifecycleManager final : public rclcpp::Node {
 public:
  using Manage = robot_ai_interfaces::srv::ManageLifecycleSkill;
  using Inspect = robot_ai_interfaces::srv::GetLifecycleSkillState;
  using ChangeState = lifecycle_msgs::srv::ChangeState;
  using GetState = lifecycle_msgs::srv::GetState;

  // Special function: Load the registry and expose serialized lifecycle operations.
  GpsrLifecycleManager() : rclcpp::Node("gpsr_lifecycle_manager") {
    const auto default_path =
        ament_index_cpp::get_package_share_directory("robot_yasmin_tasks") +
        "/config/skill_registry.yaml";
    const auto registry_path =
        declare_parameter<std::string>("registry_file", default_path);
    transition_timeout_sec_ = declare_parameter<double>("transition_timeout_sec", 60.0);
    allow_simulated_ = declare_parameter<bool>("allow_simulated_primitives", false);
    // Section: Disable master-adapter transitions when GPSR targets the persistent
    // legacy server.
    const bool manage_follow_person_adapter =
        declare_parameter<bool>("manage_follow_person_adapter", true);
    load_registry(registry_path);
    if (!manage_follow_person_adapter) {
      const auto follow = registry_.find(FOLLOW_PERSON);
      if (follow != registry_.end()) follow->second.lifecycle_nodes.clear();
    }
    std::set<std::string> nodes;
    for (const auto &[name, spec] : registry_) {
      (void)name;
      nodes.insert(spec.lifecycle_nodes.begin(), spec.lifecycle_nodes.end());
    }
    for (const auto &node : nodes) {
      state_clients_[node] = create_client<GetState>(node + "/get_state");
      change_clients_[node] = create_client<ChangeState>(node + "/change_state");
    }
    prepare_service_ = create_service<Manage>(
        GPSR_PREPARE_SKILL_SERVICE, [this](const Manage::Request::SharedPtr request,
                                           Manage::Response::SharedPtr response) {
          std::lock_guard<std::recursive_mutex> lock(operation_mutex_);
          const double timeout = request_timeout(request->timeout_sec);
          const auto result = prepare_skill(normalize(request->skill_name), timeout);
          fill_response(response, request->skill_name, result, timeout);
        });
    cleanup_service_ = create_service<Manage>(
        GPSR_CLEANUP_SKILL_SERVICE, [this](const Manage::Request::SharedPtr request,
                                           Manage::Response::SharedPtr response) {
          std::lock_guard<std::recursive_mutex> lock(operation_mutex_);
          const double timeout = request_timeout(request->timeout_sec);
          const auto result = cleanup_skill(normalize(request->skill_name), timeout);
          fill_response(response, request->skill_name, result, timeout);
        });
    recover_service_ = create_service<Manage>(
        GPSR_RECOVER_SKILL_SERVICE, [this](const Manage::Request::SharedPtr request,
                                           Manage::Response::SharedPtr response) {
          std::lock_guard<std::recursive_mutex> lock(operation_mutex_);
          const double timeout = request_timeout(request->timeout_sec);
          const auto name = normalize(request->skill_name);
          auto result = cleanup_skill(name, timeout);
          if (result.first) {
            const auto found = registry_.find(name);
            if (found != registry_.end())
              integrity_faults_.erase(found->second.conflict_group);
            result = prepare_skill(name, timeout);
          }
          fill_response(response, name, result, timeout);
        });
    state_service_ = create_service<Inspect>(
        GPSR_GET_SKILL_STATE_SERVICE, [this](const Inspect::Request::SharedPtr request,
                                             Inspect::Response::SharedPtr response) {
          std::lock_guard<std::recursive_mutex> lock(operation_mutex_);
          const auto name = normalize(request->skill_name);
          const auto found = registry_.find(name);
          if (found == registry_.end()) {
            response->success = false;
            response->message = "Unknown GPSR skill: " + name;
            response->state_json = "{}";
            return;
          }
          std::map<std::string, std::pair<std::uint8_t, std::string>> states;
          const auto error =
              group_states(found->second, transition_timeout_sec_, states);
          response->success = error.empty();
          response->message = error.empty() ? "state query complete" : error;
          response->state_json = state_payload(found->second, states).dump();
        });
    RCLCPP_INFO(get_logger(),
                "[GPSR_LIFECYCLE] manager ready with %zu ActionStep entries",
                registry_.size());
  }

 private:
  // Special function: Uppercase and trim a primitive identifier for registry lookup.
  static std::string normalize(std::string value) {
    value.erase(value.begin(),
                std::find_if(value.begin(), value.end(),
                             [](unsigned char c) { return !std::isspace(c); }));
    value.erase(std::find_if(value.rbegin(), value.rend(),
                             [](unsigned char c) { return !std::isspace(c); })
                    .base(),
                value.end());
    std::transform(value.begin(), value.end(), value.begin(), ::toupper);
    return value;
  }

  // Special function: Load the authoritative JSON-compatible YAML registry.
  void load_registry(const std::string &path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Cannot open skill registry: " + path);
    Json document;
    stream >> document;
    for (auto entry = document.at("skills").begin();
         entry != document.at("skills").end(); ++entry) {
      const auto &value = entry.value();
      SkillLifecycleSpec spec;
      spec.name = entry.key();
      spec.conflict_group = value.value("conflict_group", "");
      spec.lifecycle_nodes = value.value("lifecycle_nodes", std::vector<std::string>{});
      spec.execution_interface = value.value("execution_interface", "");
      spec.interface_type = value.value("interface_type", "");
      spec.implementation = value.value("implementation", "real");
      spec.production_enabled = value.value("production_enabled", true);
      registry_[spec.name] = std::move(spec);
    }
  }

  // Special function: Resolve positive request timeout against the configured default.
  double request_timeout(float requested) const {
    return std::max(0.1, requested > 0.0F ? static_cast<double>(requested)
                                          : transition_timeout_sec_);
  }

  // Special function: Wait for one externally spun service future.
  template <typename Future>
  static bool wait_future(Future &future, double timeout) {
    return future.wait_for(std::chrono::duration<double>(std::max(0.1, timeout))) ==
           std::future_status::ready;
  }

  // Special function: Query one lifecycle node with the existing bounded service wait.
  std::tuple<std::uint8_t, std::string, std::string> node_state(const std::string &node,
                                                                double timeout) {
    const auto client = state_clients_.at(node);
    if (!client->wait_for_service(
            std::chrono::duration<double>(std::min(2.0, timeout)))) {
      return {0, "unknown", "Lifecycle state service unavailable: " + node};
    }
    auto future = client->async_send_request(std::make_shared<GetState::Request>());
    if (!wait_future(future, timeout)) {
      return {0, "unknown", "Lifecycle state query timed out: " + node};
    }
    const auto response = future.get();
    return {response->current_state.id, response->current_state.label, ""};
  }

  // Special function: Request one lifecycle transition without retrying it.
  std::pair<bool, std::string> change_state(const std::string &node,
                                            std::uint8_t transition, double timeout) {
    const auto client = change_clients_.at(node);
    if (!client->wait_for_service(
            std::chrono::duration<double>(std::min(2.0, timeout)))) {
      return {false, "Lifecycle transition service unavailable: " + node};
    }
    auto request = std::make_shared<ChangeState::Request>();
    request->transition.id = transition;
    auto future = client->async_send_request(request);
    if (!wait_future(future, timeout)) {
      return {false, "Lifecycle transition timed out: " + node};
    }
    const auto response = future.get();
    return response->success
               ? std::make_pair(true, std::string("transition complete"))
               : std::make_pair(false, "Lifecycle transition rejected: " + node);
  }

  // Special function: Query every node belonging to one registry group.
  std::string group_states(
      const SkillLifecycleSpec &spec, double timeout,
      std::map<std::string, std::pair<std::uint8_t, std::string>> &states) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    for (const auto &node : spec.lifecycle_nodes) {
      const double remaining =
          std::chrono::duration<double>(deadline - std::chrono::steady_clock::now())
              .count();
      if (remaining <= 0.0) return "lifecycle state-query deadline expired";
      auto [id, label, error] = node_state(node, remaining);
      if (!error.empty()) return error;
      states[node] = {id, label};
    }
    return "";
  }

  // Special function: Latch cleanup uncertainty for the corresponding conflict group.
  std::pair<bool, std::string> integrity_failure(const SkillLifecycleSpec &spec,
                                                 const std::string &message) {
    const auto fault = spec.name + ": " + message;
    integrity_faults_[spec.conflict_group] = fault;
    RCLCPP_ERROR(get_logger(), "[GPSR_LIFECYCLE] integrity failure: group=%s reason=%s",
                 spec.conflict_group.c_str(), fault.c_str());
    return {false, fault};
  }

  // Special function: Deactivate, clean, and verify a group in reverse order.
  std::pair<bool, std::string> cleanup_skill(const std::string &name, double timeout) {
    const auto found = registry_.find(name);
    if (found == registry_.end()) return {false, "Unknown GPSR skill: " + name};
    const auto &spec = found->second;
    if (spec.lifecycle_nodes.empty()) {
      return {true, name + " has no managed lifecycle resources"};
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    for (auto node = spec.lifecycle_nodes.rbegin(); node != spec.lifecycle_nodes.rend();
         ++node) {
      const double remaining =
          std::chrono::duration<double>(deadline - std::chrono::steady_clock::now())
              .count();
      if (remaining <= 0.0)
        return integrity_failure(spec, "cleanup deadline expired during deactivate");
      auto [id, label, error] = node_state(*node, remaining);
      (void)label;
      if (!error.empty()) return integrity_failure(spec, error);
      if (id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_INFO(get_logger(), "[GPSR_LIFECYCLE] %s: deactivating %s", name.c_str(),
                    node->c_str());
        const auto changed = change_state(
            *node, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, remaining);
        if (!changed.first) return integrity_failure(spec, changed.second);
      }
    }
    for (auto node = spec.lifecycle_nodes.rbegin(); node != spec.lifecycle_nodes.rend();
         ++node) {
      const double remaining =
          std::chrono::duration<double>(deadline - std::chrono::steady_clock::now())
              .count();
      if (remaining <= 0.0)
        return integrity_failure(spec, "cleanup deadline expired during cleanup");
      auto [id, label, error] = node_state(*node, remaining);
      (void)label;
      if (!error.empty()) return integrity_failure(spec, error);
      if (id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_INFO(get_logger(), "[GPSR_LIFECYCLE] %s: cleaning %s", name.c_str(),
                    node->c_str());
        const auto changed = change_state(
            *node, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, remaining);
        if (!changed.first) return integrity_failure(spec, changed.second);
      } else if (id != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        return integrity_failure(
            spec, *node + " cleanup blocked in state " + std::to_string(id));
      }
    }
    std::map<std::string, std::pair<std::uint8_t, std::string>> states;
    const auto error = group_states(spec, timeout, states);
    if (!error.empty()) return integrity_failure(spec, error);
    for (const auto &[node, state] : states) {
      if (state.first != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        return integrity_failure(spec, name + " cleanup verification failed: " + node);
      }
    }
    if (prepared_skill_ == name) prepared_skill_.clear();
    RCLCPP_INFO(get_logger(), "[GPSR_LIFECYCLE] %s: UNCONFIGURED", name.c_str());
    return {true, name + " UNCONFIGURED"};
  }

  // Special function: Clean present conflicting groups before activation.
  std::pair<bool, std::string> cleanup_conflicts(const SkillLifecycleSpec &requested,
                                                 double timeout) {
    if (requested.conflict_group.empty()) return {true, "no conflict group"};
    for (const auto &[name, spec] : registry_) {
      if (name == requested.name || spec.conflict_group != requested.conflict_group ||
          spec.lifecycle_nodes.empty())
        continue;
      bool any_ready = false;
      bool non_clean = false;
      for (const auto &node : spec.lifecycle_nodes) {
        if (!state_clients_.at(node)->service_is_ready()) continue;
        any_ready = true;
        auto [id, label, error] = node_state(node, timeout);
        (void)label;
        if (!error.empty())
          return {false, "Cannot inspect conflict peer " + name + ": " + error};
        non_clean =
            non_clean || id != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
      }
      if (any_ready && non_clean) {
        const auto result = cleanup_skill(name, timeout);
        if (!result.first) {
          return {false, "Cannot prepare " + requested.name + "; conflict " + name +
                             ": " + result.second};
        }
      }
    }
    return {true, "conflicts clean"};
  }

  // Special function: Configure, activate, and verify one complete registry group.
  std::pair<bool, std::string> prepare_skill(const std::string &name, double timeout) {
    const auto found = registry_.find(name);
    if (found == registry_.end()) return {false, "Unknown GPSR skill: " + name};
    const auto &spec = found->second;
    const bool available = (spec.implementation == "real" && spec.production_enabled) ||
                           (allow_simulated_ && spec.implementation == "simulated");
    if (!available) {
      return {
          false,
          "Primitive unavailable: " + name + " implementation=" + spec.implementation +
              " production_enabled=" + (spec.production_enabled ? "true" : "false")};
    }
    const auto fault = integrity_faults_.find(spec.conflict_group);
    if (fault != integrity_faults_.end()) {
      return {false, "Lifecycle integrity fault requires recover_skill: group=" +
                         spec.conflict_group + " reason=" + fault->second};
    }
    // Section: Release conflicting motion resources even for a persistent provider.
    const auto conflict = cleanup_conflicts(spec, timeout);
    if (!conflict.first) return conflict;
    if (spec.lifecycle_nodes.empty()) {
      prepared_skill_ = name;
      return {true, name + " uses persistent or no managed lifecycle resources"};
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    for (const auto &node : spec.lifecycle_nodes) {
      const double remaining =
          std::chrono::duration<double>(deadline - std::chrono::steady_clock::now())
              .count();
      if (remaining <= 0.0) return {false, "prepare deadline expired during configure"};
      auto [id, label, error] = node_state(node, remaining);
      (void)label;
      if (!error.empty()) return {false, name + " unavailable: " + error};
      if (id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        RCLCPP_INFO(get_logger(), "[GPSR_LIFECYCLE] %s: configuring %s", name.c_str(),
                    node.c_str());
        const auto changed = change_state(
            node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, remaining);
        if (!changed.first) {
          cleanup_skill(name, std::max(0.1, remaining));
          return {false, name + " prepare failed: " + changed.second};
        }
      } else if (id != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
                 id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        return {false, node + " cannot configure from state " + std::to_string(id)};
      }
    }
    for (const auto &node : spec.lifecycle_nodes) {
      const double remaining =
          std::chrono::duration<double>(deadline - std::chrono::steady_clock::now())
              .count();
      auto [id, label, error] = node_state(node, remaining);
      (void)label;
      if (!error.empty()) return {false, error};
      if (id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_INFO(get_logger(), "[GPSR_LIFECYCLE] %s: activating %s", name.c_str(),
                    node.c_str());
        const auto changed = change_state(
            node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, remaining);
        if (!changed.first) {
          cleanup_skill(name, std::max(0.1, remaining));
          return {false, name + " prepare failed: " + changed.second};
        }
      }
    }
    std::map<std::string, std::pair<std::uint8_t, std::string>> states;
    const auto error = group_states(spec, timeout, states);
    if (!error.empty()) return {false, error};
    for (const auto &[node, state] : states) {
      if (state.first != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        cleanup_skill(name, timeout);
        return {false, name + " activation verification failed: " + node};
      }
    }
    prepared_skill_ = name;
    RCLCPP_INFO(get_logger(), "[GPSR_LIFECYCLE] %s: ACTIVE", name.c_str());
    return {true, name + " ACTIVE"};
  }

  // Special function: Build the stable state JSON response contract.
  Json state_payload(
      const SkillLifecycleSpec &spec,
      const std::map<std::string, std::pair<std::uint8_t, std::string>> &states) const {
    Json nodes = Json::object();
    for (const auto &[node, state] : states) {
      nodes[node] = Json{{"id", state.first}, {"label", state.second}};
    }
    Json faults = Json::object();
    for (const auto &[group, reason] : integrity_faults_) faults[group] = reason;
    const auto fault = integrity_faults_.find(spec.conflict_group);
    return Json{
        {"skill_name", spec.name},
        {"conflict_group", spec.conflict_group},
        {"execution_interface", spec.execution_interface},
        {"interface_type", spec.interface_type},
        {"implementation", spec.implementation},
        {"production_enabled", spec.production_enabled},
        {"prepared_skill", prepared_skill_},
        {"integrity_fault_group",
         fault == integrity_faults_.end() ? "" : spec.conflict_group},
        {"integrity_fault", fault == integrity_faults_.end() ? "" : fault->second},
        {"integrity_faults", faults},
        {"nodes", nodes}};
  }

  // Special function: Fill operation response with best-effort actual states.
  void fill_response(const std::shared_ptr<Manage::Response> &response,
                     const std::string &name,
                     const std::pair<bool, std::string> &result, double timeout) {
    const auto normalized = normalize(name);
    const auto found = registry_.find(normalized);
    response->success = result.first;
    response->message = result.second;
    if (found == registry_.end()) {
      response->state_json = "{}";
      return;
    }
    std::map<std::string, std::pair<std::uint8_t, std::string>> states;
    group_states(found->second, timeout, states);
    response->state_json = state_payload(found->second, states).dump();
  }

  double transition_timeout_sec_{60.0};
  bool allow_simulated_{false};
  std::recursive_mutex operation_mutex_;
  std::map<std::string, SkillLifecycleSpec> registry_;
  std::map<std::string, std::string> integrity_faults_;
  std::string prepared_skill_;
  std::map<std::string, rclcpp::Client<GetState>::SharedPtr> state_clients_;
  std::map<std::string, rclcpp::Client<ChangeState>::SharedPtr> change_clients_;
  rclcpp::Service<Manage>::SharedPtr prepare_service_;
  rclcpp::Service<Manage>::SharedPtr cleanup_service_;
  rclcpp::Service<Manage>::SharedPtr recover_service_;
  rclcpp::Service<Inspect>::SharedPtr state_service_;
};

}  // namespace robot_yasmin_tasks

// Section: Process startup and lifecycle-service callback execution.
int main(int argc, char **argv) {
  // Section: Preserve the four-thread reentrant lifecycle executor.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_yasmin_tasks::GpsrLifecycleManager>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
