// Section: ActionQueue executor that creates, runs, and reports GPSR skill state
// machines.
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_ai_interfaces/action/execute_action_queue.hpp>
#include <set>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <vector>

#include "robot_yasmin_tasks/clients.hpp"
#include "robot_yasmin_tasks/runtime.hpp"
#include "robot_yasmin_tasks/skills.hpp"

namespace robot_yasmin_tasks {

// Section: ActionQueue admission, lifecycle coordination, state-machine execution, and
// status reporting.
class GpsrExecutor final : public rclcpp::Node {
 public:
  using Action = robot_ai_interfaces::action::ExecuteActionQueue;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  // Special function: Create persistent clients, status publisher, and queue action
  // server.
  GpsrExecutor() : rclcpp::Node("gpsr_executor") {
    declare_parameter<std::string>("skill_registry_file", "");
    declare_parameter<bool>("allow_simulated_primitives", false);
    const auto pose_action =
        declare_parameter<std::string>("navigate_to_pose_action", "/navigate_to_pose");
    const auto named_action = declare_parameter<std::string>(
        "navigate_to_named_pose_action", "/nav/navigate_named_pose");
    // Section: Select one concrete PersonFollow provider by its unique action endpoint.
    const auto follow_action =
        declare_parameter<std::string>("follow_person_action", "/person_follow/master");
    const auto person_search_config =
        declare_parameter<std::string>("find_person_search_config_file", "");
    const auto object_search_config =
        declare_parameter<std::string>("find_object_search_config_file", "");
    const auto lifecycle_timeout =
        declare_parameter<double>("lifecycle_transition_timeout_sec", 60.0);
    const auto lifecycle_margin =
        declare_parameter<double>("lifecycle_operation_timeout_margin_sec", 10.0);
    // Section: Keep STT inactive after TTS so final robot audio cannot become a reply.
    const auto post_tts_guard_ms =
        std::max<std::int64_t>(0, declare_parameter<std::int64_t>("post_tts_guard_ms", 400));
    declare_parameter<double>("startup_dependency_poll_sec", 5.0);
    const auto status_topic = declare_parameter<std::string>(
        "queue_status_topic", "/gpsr/action_queue_status_json");
    rclcpp::QoS status_qos(1);
    status_qos.reliable().transient_local();
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, status_qos);

    // Section: Client lifetimes match the persistent Python executor node.
    auto self = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    clients_.speech = std::make_shared<SpeechClient>(self, false);
    // Section: TTS completion and its echo guard finish before a dialogue can trigger STT.
    clients_.dialogue = std::make_shared<DialogueClient>(
        self, clients_.speech, false,
        std::chrono::milliseconds{post_tts_guard_ms});
    // Section: Fixed-information questions use their own correlated provider state.
    clients_.fixed_information =
        std::make_shared<FixedInformationClient>(self, clients_.speech, false);
    clients_.navigation =
        std::make_shared<Nav2Client>(self, false, named_action, pose_action);
    clients_.perception = std::make_shared<PerceptionClient>(self, false);
    clients_.follow = std::make_shared<FollowPersonClient>(self, follow_action);
    clients_.remote = std::make_shared<RemoteSkillClient>(self);
    clients_.manipulation = std::make_shared<ManipulationClient>(self, false);
    clients_.guide = std::make_shared<GuidePersonClient>(self, false);
    load_person_search_policy(person_search_config);
    load_object_search_policy(object_search_config);
    lifecycle_ = std::make_shared<LifecycleResourceClient>(self, lifecycle_timeout,
                                                           lifecycle_margin);

    action_server_ = rclcpp_action::create_server<Action>(
        this, "execute_action_queue",
        std::bind(&GpsrExecutor::goal_callback, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&GpsrExecutor::cancel_callback, this, std::placeholders::_1),
        std::bind(&GpsrExecutor::accepted_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
        get_logger(),
        "GPSR executor waiting for one goal at a time on /execute_action_queue.");
  }

 private:
  // Special function: Load the shared person-search route without changing safe
  // defaults.
  void load_person_search_policy(const std::string &config_file) {
    // Section: An empty path intentionally selects the compiled compatibility policy.
    if (config_file.empty()) return;
    try {
      const auto root = YAML::LoadFile(config_file);
      const auto search = root["person_search"];
      const auto navigation = root["navigation"];
      if (search["global_waypoints"]) {
        clients_.person_search.global_waypoints =
            search["global_waypoints"].as<std::vector<std::string>>();
      }
      if (search["observation_timeout_sec"]) {
        clients_.person_search.observation_timeout_sec =
            search["observation_timeout_sec"].as<double>();
      }
      if (search["mission_timeout_sec"]) {
        clients_.person_search.mission_timeout_sec =
            search["mission_timeout_sec"].as<double>();
      }
      // Section: Keep person rescans bounded and separate from queue-level retries.
      const auto retry = search["retry"];
      if (retry && retry["max_not_found_retries"]) {
        const auto configured_retries = retry["max_not_found_retries"].as<long long>();
        constexpr std::uint32_t kMaximumPersonRescans = 1U;
        if (configured_retries < 0) {
          RCLCPP_WARN(get_logger(),
                      "Ignoring negative FIND_PERSON max_not_found_retries=%lld.",
                      configured_retries);
        } else {
          clients_.person_search.max_not_found_retries = static_cast<std::uint32_t>(
              std::min<unsigned long long>(
                  static_cast<unsigned long long>(configured_retries),
                  kMaximumPersonRescans));
        }
      }
      if (search["scan_timeout_sec"]) {
        clients_.person_search.scan_timeout_sec =
            search["scan_timeout_sec"].as<double>();
      }
      if (navigation["waypoint_timeout_sec"]) {
        clients_.person_search.navigation_timeout_sec =
            navigation["waypoint_timeout_sec"].as<double>();
      }
      std::vector<double> headings;
      const auto scan = search["scan"];
      for (const auto *tier :
           {"fast_angles_deg", "extended_angles_deg", "exhaustive_angles_deg"}) {
        if (!scan[tier]) continue;
        const auto values = scan[tier].as<std::vector<double>>();
        headings.insert(headings.end(), values.begin(), values.end());
      }
      if (!headings.empty() && headings.front() == 0.0)
        headings.erase(headings.begin());
      if (!headings.empty()) clients_.person_search.scan_headings_deg = headings;
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "Unable to load person-search policy '%s': %s",
                  config_file.c_str(), error.what());
    }
  }

  // Special function: Load the object-search room route and scan policy without
  // changing safe compiled defaults when configuration is unavailable.
  void load_object_search_policy(const std::string &config_file) {
    // Section: An empty path intentionally selects the compiled compatibility policy.
    if (config_file.empty()) return;
    try {
      const auto root = YAML::LoadFile(config_file);
      std::vector<std::string> waypoints;
      const auto rooms = root["rooms"];
      if (rooms && rooms.IsMap()) {
        for (const auto &room : rooms) {
          const auto viewpoints = room.second["viewpoints"];
          if (!viewpoints || !viewpoints.IsSequence()) continue;
          for (const auto &viewpoint : viewpoints) {
            const auto name = viewpoint.as<std::string>();
            if (std::find(waypoints.begin(), waypoints.end(), name) ==
                waypoints.end()) {
              waypoints.push_back(name);
            }
          }
        }
      }
      if (!waypoints.empty()) clients_.object_search.global_waypoints = waypoints;

      // Section: The semantic retry is deliberately separate from ActionStep max_retries.
      const auto retry = root["retry"];
      if (retry && !retry.IsMap()) {
        RCLCPP_WARN(get_logger(),
                    "Ignoring malformed FIND_OBJECT retry policy; expected a map.");
      }
      const auto configured_retries = retry && retry.IsMap()
                                          ? retry["max_not_found_retries"]
                                          : YAML::Node{};
      if (configured_retries) {
        constexpr std::uint32_t kMaximumConfiguredNotFoundRetries = 10U;
        try {
          const auto parsed_retries = configured_retries.as<long long>();
          if (parsed_retries < 0) {
            RCLCPP_WARN(get_logger(),
                        "Ignoring negative FIND_OBJECT max_not_found_retries=%lld; "
                        "keeping %u.",
                        parsed_retries,
                        clients_.object_search.max_not_found_retries);
          } else if (static_cast<unsigned long long>(parsed_retries) >
                     kMaximumConfiguredNotFoundRetries) {
            clients_.object_search.max_not_found_retries =
                kMaximumConfiguredNotFoundRetries;
            RCLCPP_WARN(
                get_logger(),
                "Clamping FIND_OBJECT max_not_found_retries=%lld to %u.",
                parsed_retries, kMaximumConfiguredNotFoundRetries);
          } else {
            clients_.object_search.max_not_found_retries =
                static_cast<std::uint32_t>(parsed_retries);
          }
        } catch (const YAML::Exception &error) {
          RCLCPP_WARN(get_logger(),
                      "Ignoring invalid FIND_OBJECT max_not_found_retries: %s",
                      error.what());
        }
      }

      // Section: The installed object policy provides tiered scan headings and one
      // bounded rotation budget.
      const auto scan = root["scan"];
      if (scan && scan["action_timeout_sec"]) {
        clients_.object_search.scan_timeout_sec =
            scan["action_timeout_sec"].as<double>();
      }
      std::vector<double> headings;
      for (const auto *tier :
           {"fast_angles_deg", "extended_angles_deg", "exhaustive_angles_deg"}) {
        if (!scan || !scan[tier]) continue;
        const auto values = scan[tier].as<std::vector<double>>();
        headings.insert(headings.end(), values.begin(), values.end());
      }
      if (!headings.empty() && headings.front() == 0.0) {
        headings.erase(headings.begin());
      }
      if (!headings.empty()) clients_.object_search.scan_headings_deg = headings;

      // Section: A local miss waits only for the configured confirmation window so
      // global searches can continue through every waypoint within the mission budget.
      const auto verification = root["verification"];
      if (verification && verification["confirmation_timeout_sec"]) {
        clients_.object_search.observation_timeout_sec =
            verification["confirmation_timeout_sec"].as<double>();
      }

      // Section: Reuse the object approach navigation budget for named waypoint
      // travel because both operations depend on the same Nav2 availability.
      const auto approach = root["approach"];
      if (approach && approach["navigation_timeout_sec"]) {
        clients_.object_search.navigation_timeout_sec =
            approach["navigation_timeout_sec"].as<double>();
      }
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "Unable to load object-search policy '%s': %s",
                  config_file.c_str(), error.what());
    }
  }

  // Special function: Admit only one queue goal at a time.
  rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &,
                                            std::shared_ptr<const Action::Goal>) {
    std::lock_guard<std::mutex> lock(admission_mutex_);
    if (busy_) return rclcpp_action::GoalResponse::REJECT;
    busy_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Special function: Accept queue cancellation for cooperative child polling.
  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Special function: Execute the accepted queue on a worker as in the Python action
  // server.
  void accepted_callback(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  // Special function: Publish one authoritative queue status snapshot.
  void publish_status(const std::shared_ptr<GoalHandle> &goal_handle,
                      const std::string &phase, std::size_t index,
                      const std::string &action_type,
                      const std::vector<SkillResultData> &results,
                      const std::string &outcome = "") {
    Json result_values = Json::array();
    for (const auto &result : results) result_values.push_back(result_to_json(result));
    Json payload{{"phase", phase},
                 {"current_index", index},
                 {"current_action_type", action_type},
                 {"outcome", outcome},
                 {"results", result_values}};
    std_msgs::msg::String status;
    status.data = payload.dump();
    status_pub_->publish(status);
    if (goal_handle && goal_handle->is_active()) {
      auto feedback = std::make_shared<Action::Feedback>();
      feedback->current_index = static_cast<std::uint32_t>(index);
      feedback->current_action_type = action_type;
      feedback->status = phase;
      feedback->status_json = status.data;
      goal_handle->publish_feedback(feedback);
    }
  }

  // Special function: Run prerequisite checks, lifecycle barriers, skill dispatch, and
  // retries.
  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto finish_admission = [this]() {
      std::lock_guard<std::mutex> lock(admission_mutex_);
      busy_ = false;
    };
    const auto queue = goal_handle->get_goal()->queue;
    auto blackboard = std::make_shared<Blackboard>();
    blackboard->set<std::function<bool()>>(
        bb::CANCEL_REQUESTED, [goal_handle]() { return goal_handle->is_canceling(); });
    blackboard->set<std::string>(bb::CURRENT_TASK_ID, queue.task_id);
    blackboard->set<Json>(bb::RESULT_VARIABLES, Json::object());
    blackboard->set<std::vector<std::string>>(bb::COMPLETED_STEP_IDS, {});
    blackboard->set<std::map<std::string, int>>(bb::RECOVERY_ATTEMPTS, {});
    std::vector<SkillResultData> results;
    std::set<std::string> completed;
    std::string terminal = SUCCESS;
    publish_status(goal_handle, "QUEUE_STARTED", 0, "", results);
    for (std::size_t index = 0; index < queue.steps.size(); ++index) {
      const auto action = action_from_msg(queue.steps[index]);
      if (goal_handle->is_canceling()) {
        terminal = CANCELED;
        break;
      }
      bool prerequisites = true;
      for (const auto &dependency : action.requires_success) {
        if (completed.count(dependency) == 0U) prerequisites = false;
      }
      const auto variables =
          bb_get<Json>(blackboard, bb::RESULT_VARIABLES, Json::object());
      for (const auto &key : action.requires_data) {
        if (!variables.contains(key)) prerequisites = false;
      }
      if (!prerequisites) {
        results.push_back(make_skill_result(action.action_type, INVALID_INPUT,
                                            "Action prerequisites are not satisfied.",
                                            Json::object(), false, true));
        terminal = INVALID_INPUT;
        break;
      }
      blackboard->set<ActionStepData>(bb::CURRENT_ACTION, action);
      publish_status(goal_handle, "PREPARING_SKILL", index, action.action_type,
                     results);
      auto prepared = lifecycle_->start(action.action_type);
      if (!prepared.first) {
        results.push_back(make_skill_result(action.action_type, SKILL_UNAVAILABLE,
                                            prepared.second, Json::object(), false,
                                            true));
        terminal = SKILL_UNAVAILABLE;
        break;
      }
      SkillResultData result;
      unsigned attempt = 0;
      while (true) {
        publish_status(goal_handle, "EXECUTING_SKILL", index, action.action_type,
                       results);
        try {
          auto skill = make_skill_state_machine(action.action_type, clients_);
          const auto outcome = (*skill)(blackboard);
          result = bb_get<SkillResultData>(
              blackboard, bb::LAST_SKILL_RESULT,
              make_skill_result(action.action_type, outcome,
                                "Skill returned no structured result.", Json::object(),
                                true));
        } catch (const std::exception &error) {
          result = make_skill_result(action.action_type, SKILL_UNAVAILABLE,
                                     error.what(), Json::object(), false, true);
        }
        const auto cleaned = lifecycle_->stop(action.action_type);
        if (!cleaned.first) {
          result = make_skill_result(action.action_type, FATAL_FAILURE, cleaned.second,
                                     result.payload, false, true);
        }
        // Section: FIND_OBJECT owns its NOT_FOUND retry inside the compound search
        // mission; never restart the full lifecycle-backed skill here.
        const bool object_miss_owned_by_search_policy =
            action.action_type == FIND_OBJECT && result.outcome == NOT_FOUND;
        if (object_miss_owned_by_search_policy || !is_recoverable(result.outcome) ||
            attempt >= action.max_retries)
          break;
        ++attempt;
        const auto recovered = lifecycle_->recover(action.action_type);
        if (!recovered.first) {
          result = make_skill_result(action.action_type, FATAL_FAILURE,
                                     recovered.second, result.payload, false, true);
          break;
        }
      }
      results.push_back(result);
      if (result.outcome == SUCCESS) {
        if (!action.step_id.empty()) completed.insert(action.step_id);
        if (!action.result_variable.empty()) {
          auto updated = bb_get<Json>(blackboard, bb::RESULT_VARIABLES, Json::object());
          updated[action.result_variable] = result.payload;
          blackboard->set<Json>(bb::RESULT_VARIABLES, updated);
        }
        publish_status(goal_handle, "STEP_COMMITTED", index, action.action_type,
                       results, result.outcome);
        continue;
      }
      terminal = result.outcome;
      break;
    }
    auto action_result = std::make_shared<Action::Result>();
    action_result->success = terminal == SUCCESS;
    action_result->outcome = terminal;
    for (const auto &result : results) {
      action_result->results.push_back(result_to_msg(result));
    }
    publish_status(goal_handle, "QUEUE_FINISHED", queue.steps.size(), "", results,
                   terminal);
    if (terminal == SUCCESS)
      goal_handle->succeed(action_result);
    // Section: A declined spoken confirmation is a business-level CANCELED result,
    // not a ROS action cancellation. rclcpp permits canceled() only after the client
    // has requested cancellation and the goal is therefore in CANCELING state.
    else if (terminal == CANCELED && goal_handle->is_canceling())
      goal_handle->canceled(action_result);
    else
      goal_handle->abort(action_result);
    finish_admission();
  }

  std::mutex admission_mutex_;
  bool busy_{false};
  SkillClients clients_;
  std::shared_ptr<LifecycleResourceClient> lifecycle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
};

}  // namespace robot_yasmin_tasks

// Section: Process startup and multithreaded ROS callback execution.
int main(int argc, char **argv) {
  // Section: Preserve the executor's four-thread ROS callback capacity.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_yasmin_tasks::GpsrExecutor>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
