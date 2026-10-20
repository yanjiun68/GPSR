// Section: ActionQueue executor that creates, runs, and reports GPSR skill state
// machines.
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_ai_interfaces/action/execute_action_queue.hpp>
#include <set>
#include <sstream>
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
  // Section: Preflight metadata binds each required data key to its nearest prior
  // producer while retaining an exact structural-validation failure location.
  struct QueueValidation {
    bool valid{true};
    std::size_t error_index{0U};
    std::string error_action_type;
    std::string message;
    std::vector<std::map<std::string, std::string>> data_producers;
  };

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

  // Special function: Reject malformed identities and prerequisite references before
  // any lifecycle transition or robot skill execution can begin.
  QueueValidation validate_queue(const std::vector<ActionStepData> &actions,
                                 const Json &initial_variables) const {
    QueueValidation validation;
    validation.data_producers.resize(actions.size());
    std::set<std::string> prior_step_ids;
    std::map<std::string, std::string> latest_data_producer;
    for (std::size_t index = 0; index < actions.size(); ++index) {
      const auto &action = actions[index];
      auto reject = [&](const std::string &message) {
        validation.valid = false;
        validation.error_index = index;
        validation.error_action_type = action.action_type;
        validation.message = message;
      };
      if (action.step_id.empty()) {
        reject("ActionQueue contains an empty step_id at index " +
               std::to_string(index) + ".");
        return validation;
      }
      if (prior_step_ids.count(action.step_id) != 0U) {
        reject("ActionQueue contains duplicate step_id '" + action.step_id + "'.");
        return validation;
      }
      for (const auto &dependency : action.requires_success) {
        if (dependency.empty() || prior_step_ids.count(dependency) == 0U) {
          reject("Step '" + action.step_id +
                 "' references unknown or future requires_success step '" +
                 dependency + "'.");
          return validation;
        }
      }
      for (const auto &key : action.requires_data) {
        if (key.empty()) {
          reject("Step '" + action.step_id +
                 "' contains an empty requires_data key.");
          return validation;
        }
        if (initial_variables.contains(key)) {
          validation.data_producers[index][key] = "";
          continue;
        }
        const auto producer = latest_data_producer.find(key);
        if (producer == latest_data_producer.end()) {
          reject("Step '" + action.step_id + "' requires data key '" + key +
                 "' with no prior producer or initial blackboard value.");
          return validation;
        }
        validation.data_producers[index][key] = producer->second;
      }
      prior_step_ids.insert(action.step_id);
      if (!action.result_variable.empty()) {
        latest_data_producer[action.result_variable] = action.step_id;
      }
    }
    return validation;
  }

  // Special function: Join dependency diagnostics without losing any failed producer.
  static std::string join_diagnostics(const std::vector<std::string> &diagnostics) {
    std::ostringstream stream;
    for (std::size_t index = 0; index < diagnostics.size(); ++index) {
      if (index != 0U) stream << "; ";
      stream << diagnostics[index];
    }
    return stream.str();
  }

  // Special function: Publish one authoritative queue status snapshot with progress,
  // correlated step identities, and aggregate result counts.
  void publish_status(const std::shared_ptr<GoalHandle> &goal_handle,
                      const std::string &phase, std::size_t index,
                      std::size_t total_steps, const std::string &step_id,
                      const std::string &action_type,
                      const std::vector<SkillResultData> &results,
                      const std::vector<std::string> &result_step_ids,
                      const std::string &outcome = "") {
    Json result_values = Json::array();
    std::size_t successful_steps = 0U;
    std::size_t skipped_steps = 0U;
    for (std::size_t result_index = 0; result_index < results.size(); ++result_index) {
      auto value = result_to_json(results[result_index]);
      if (result_index < result_step_ids.size()) {
        value["step_id"] = result_step_ids[result_index];
      }
      result_values.push_back(value);
      successful_steps += results[result_index].outcome == SUCCESS ? 1U : 0U;
      skipped_steps +=
          results[result_index].outcome == SKIPPED_DEPENDENCY_FAILED ? 1U : 0U;
    }
    Json payload{{"phase", phase},
                 {"current_index", index},
                 {"processed_count", results.size()},
                 {"total_steps", total_steps},
                 {"step_id", step_id},
                 {"current_action_type", action_type},
                 {"outcome", outcome},
                 {"results", result_values},
                 {"summary",
                  {{"successful_steps", successful_steps},
                   {"failed_steps", results.size() - successful_steps - skipped_steps},
                   {"skipped_steps", skipped_steps},
                   {"total_steps", total_steps}}}};
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

  // Special function: Run validated dependency checks, lifecycle barriers, skill
  // dispatch, retries, and best-effort sequential aggregation.
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
    std::vector<std::string> result_step_ids;
    std::vector<ActionStepData> actions;
    actions.reserve(queue.steps.size());
    for (const auto &step : queue.steps) actions.push_back(action_from_msg(step));
    std::map<std::string, std::string> step_outcomes;
    for (const auto &action : actions) {
      step_outcomes[action.step_id] = "not_executed";
    }
    std::string terminal = SUCCESS;
    bool abnormal_termination = false;
    const auto initial_variables =
        bb_get<Json>(blackboard, bb::RESULT_VARIABLES, Json::object());
    const auto validation = validate_queue(actions, initial_variables);
    publish_status(goal_handle, "QUEUE_STARTED", 0, actions.size(), "", "", results,
                   result_step_ids);

    // Section: Structural queue errors terminate before the first lifecycle prepare.
    if (!validation.valid) {
      const auto action_type = validation.error_action_type.empty()
                                   ? std::string{"ACTION_QUEUE"}
                                   : validation.error_action_type;
      results.push_back(make_skill_result(action_type, INVALID_INPUT,
                                          validation.message, Json::object(), false,
                                          true));
      result_step_ids.push_back(
          validation.error_index < actions.size()
              ? actions[validation.error_index].step_id
              : std::string{});
      terminal = INVALID_INPUT;
      abnormal_termination = true;
      publish_status(goal_handle, "STEP_FAILED", validation.error_index,
                     actions.size(), result_step_ids.back(), action_type, results,
                     result_step_ids, terminal);
    }

    for (std::size_t index = 0; validation.valid && index < actions.size(); ++index) {
      const auto &action = actions[index];
      if (goal_handle->is_canceling()) {
        terminal = CANCELED;
        abnormal_termination = true;
        break;
      }

      // Section: A valid runtime dependency failure skips only its consumer; it does
      // not prepare lifecycle resources or dispatch a skill goal.
      std::vector<std::string> failed_dependencies;
      for (const auto &dependency : action.requires_success) {
        const auto dependency_outcome = step_outcomes.at(dependency);
        if (dependency_outcome != SUCCESS) {
          failed_dependencies.push_back("step " + dependency + " (" +
                                        dependency_outcome + ")");
        }
      }
      const auto variables =
          bb_get<Json>(blackboard, bb::RESULT_VARIABLES, Json::object());
      for (const auto &key : action.requires_data) {
        const auto &producer = validation.data_producers[index].at(key);
        if (!producer.empty() && step_outcomes.at(producer) != SUCCESS) {
          failed_dependencies.push_back("data " + key + " from step " + producer +
                                        " (" + step_outcomes.at(producer) + ")");
        } else if (!variables.contains(key)) {
          // Section: A successful producer that omitted its declared result is a
          // fatal execution contract violation, not an ordinary dependency skip.
          const auto message = producer.empty()
                                   ? "Required initial data key '" + key +
                                         "' disappeared before step '" +
                                         action.step_id + "'."
                                   : "Required data key '" + key + "' is missing even "
                                     "though producer step '" + producer +
                                         "' reported success.";
          auto result = make_skill_result(action.action_type, FATAL_FAILURE, message,
                                          Json::object(), false, true);
          results.push_back(result);
          result_step_ids.push_back(action.step_id);
          step_outcomes[action.step_id] = result.outcome;
          terminal = FATAL_FAILURE;
          abnormal_termination = true;
          publish_status(goal_handle, "STEP_FAILED", index, actions.size(),
                         action.step_id, action.action_type, results,
                         result_step_ids, result.outcome);
          break;
        }
      }
      if (abnormal_termination) break;
      if (!failed_dependencies.empty()) {
        const auto message = "Skipped because required dependencies did not succeed: " +
                             join_diagnostics(failed_dependencies) + ".";
        auto result = make_skill_result(action.action_type,
                                        SKIPPED_DEPENDENCY_FAILED, message,
                                        Json::object(), false, false);
        results.push_back(result);
        result_step_ids.push_back(action.step_id);
        step_outcomes[action.step_id] = result.outcome;
        publish_status(goal_handle, "STEP_SKIPPED", index, actions.size(),
                       action.step_id, action.action_type, results, result_step_ids,
                       result.outcome);
        continue;
      }
      blackboard->set<ActionStepData>(bb::CURRENT_ACTION, action);
      publish_status(goal_handle, "PREPARING_SKILL", index, actions.size(),
                     action.step_id, action.action_type, results, result_step_ids);
      auto prepared = lifecycle_->start(action.action_type);
      if (!prepared.first) {
        auto result = make_skill_result(action.action_type, SKILL_UNAVAILABLE,
                                        prepared.second,
                                        failure_payload(
                                            LIFECYCLE_SKILL_UNAVAILABLE,
                                            "lifecycle_prepare",
                                            Json{{"skill", action.action_type}}),
                                        false, true);
        results.push_back(result);
        result_step_ids.push_back(action.step_id);
        step_outcomes[action.step_id] = result.outcome;
        terminal = SKILL_UNAVAILABLE;
        abnormal_termination = true;
        publish_status(goal_handle, "STEP_FAILED", index, actions.size(),
                       action.step_id, action.action_type, results, result_step_ids,
                       result.outcome);
        break;
      }
      SkillResultData result;
      unsigned attempt = 0;
      while (true) {
        publish_status(goal_handle, "EXECUTING_SKILL", index, actions.size(),
                       action.step_id, action.action_type, results, result_step_ids);
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
                                     error.what(),
                                     failure_payload(
                                         SKILL_UNAVAILABLE_FAILURE,
                                         "skill_construction",
                                         Json{{"skill", action.action_type}}),
                                     false, true);
        }
        const auto cleaned = lifecycle_->stop(action.action_type);
        if (!cleaned.first) {
          result = make_skill_result(action.action_type, FATAL_FAILURE, cleaned.second,
                                     failure_payload(
                                         LIFECYCLE_SKILL_UNAVAILABLE,
                                         "lifecycle_cleanup",
                                         Json{{"skill", action.action_type},
                                              {"prior_result", result.payload}}),
                                     false, true);
        }
        // Section: FIND_OBJECT owns its NOT_FOUND retry inside the compound search
        // mission; never restart the full lifecycle-backed skill here.
        const bool object_miss_owned_by_search_policy =
            action.action_type == FIND_OBJECT && result.outcome == NOT_FOUND;
        if (object_miss_owned_by_search_policy || result.fatal ||
            !is_recoverable(result.outcome) ||
            attempt >= action.max_retries)
          break;
        ++attempt;
        const auto recovered = lifecycle_->recover(action.action_type);
        if (!recovered.first) {
          result = make_skill_result(action.action_type, FATAL_FAILURE,
                                     recovered.second,
                                     failure_payload(
                                         LIFECYCLE_SKILL_UNAVAILABLE,
                                         "lifecycle_recovery",
                                         Json{{"skill", action.action_type},
                                              {"prior_result", result.payload}}),
                                     false, true);
          break;
        }
      }
      results.push_back(result);
      result_step_ids.push_back(action.step_id);
      step_outcomes[action.step_id] = result.outcome;
      if (result.fatal || is_fatal(result.outcome)) {
        terminal = result.outcome;
        abnormal_termination = true;
        publish_status(goal_handle, "STEP_FAILED", index, actions.size(),
                       action.step_id, action.action_type, results, result_step_ids,
                       result.outcome);
        break;
      }
      if (result.outcome == SUCCESS) {
        auto completed = bb_get<std::vector<std::string>>(
            blackboard, bb::COMPLETED_STEP_IDS, {});
        completed.push_back(action.step_id);
        blackboard->set<std::vector<std::string>>(bb::COMPLETED_STEP_IDS, completed);
        if (!action.result_variable.empty()) {
          auto updated = bb_get<Json>(blackboard, bb::RESULT_VARIABLES, Json::object());
          updated[action.result_variable] = result.payload;
          blackboard->set<Json>(bb::RESULT_VARIABLES, updated);
        }
        publish_status(goal_handle, "STEP_COMMITTED", index, actions.size(),
                       action.step_id, action.action_type, results, result_step_ids,
                       result.outcome);
        continue;
      }
      publish_status(goal_handle, "STEP_FAILED", index, actions.size(), action.step_id,
                     action.action_type, results, result_step_ids, result.outcome);
      // Section: Cleanup has succeeded and retries are exhausted, so the ordinary
      // task failure is committed before the next independent ordered step.
    }

    // Section: Only a complete normal traversal receives the aggregate business
    // outcome; fatal diagnostics retain their specific terminal outcome.
    if (!abnormal_termination) terminal = aggregate_queue_outcome(results);
    auto action_result = std::make_shared<Action::Result>();
    action_result->success = terminal == SUCCESS;
    action_result->outcome = terminal;
    for (const auto &result : results) {
      action_result->results.push_back(result_to_msg(result));
    }
    publish_status(goal_handle, "QUEUE_FINISHED", results.size(), actions.size(), "",
                   "", results, result_step_ids, terminal);
    // Section: SUCCESS, PARTIAL_SUCCESS, and FAILED are normal executor traversal
    // completions even though only SUCCESS sets the business success flag.
    if (!abnormal_termination)
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
