#ifndef ROBOT_YASMIN_TASKS__RUNTIME_HPP_
#define ROBOT_YASMIN_TASKS__RUNTIME_HPP_

// Section: Standard and ROS types shared by every converted deterministic component.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>
#include <robot_ai_interfaces/msg/action_step.hpp>
#include <robot_ai_interfaces/msg/skill_result.hpp>
#include <yasmin/blackboard.hpp>

namespace robot_yasmin_tasks {

using Json = nlohmann::json;
using Blackboard = yasmin::Blackboard;

// Section: Preserve Python outcome strings byte-for-byte.
inline constexpr char SUCCESS[] = "success";
inline constexpr char FAILED[] = "failed";
inline constexpr char CANCELED[] = "canceled";
inline constexpr char TIMEOUT[] = "timeout";
inline constexpr char INVALID_INPUT[] = "invalid_input";
inline constexpr char NOT_FOUND[] = "not_found";
inline constexpr char NEED_CLARIFICATION[] = "need_clarification";
inline constexpr char RECOVERABLE_FAILURE[] = "recoverable_failure";
inline constexpr char FATAL_FAILURE[] = "fatal_failure";
inline constexpr char PERSON_LOST[] = "person_lost";
inline constexpr char SKILL_UNAVAILABLE[] = "skill_unavailable";

// Section: Preserve canonical primitive identifiers and lifecycle endpoint names.
inline constexpr char NAVIGATE[] = "NAVIGATE";
inline constexpr char FIND_OBJECT[] = "FIND_OBJECT";
inline constexpr char FIND_PERSON[] = "FIND_PERSON";
inline constexpr char FIND_PERSON_BY_ATTRIBUTE[] = "FIND_PERSON_BY_ATTRIBUTE";
inline constexpr char COUNT_OBJECTS[] = "COUNT_OBJECTS";
inline constexpr char COUNT_PEOPLE[] = "COUNT_PEOPLE";
inline constexpr char EXTRACT_PERSON_INFO[] = "EXTRACT_PERSON_INFO";
inline constexpr char PICK_OBJECT[] = "PICK_OBJECT";
inline constexpr char PLACE_OBJECT[] = "PLACE_OBJECT";
inline constexpr char HANDOVER_OBJECT[] = "HANDOVER_OBJECT";
inline constexpr char FOLLOW_PERSON[] = "FOLLOW_PERSON";
inline constexpr char GUIDE_PERSON[] = "GUIDE_PERSON";
inline constexpr char SAY[] = "SAY";
inline constexpr char ASK_AND_LISTEN[] = "ASK_AND_LISTEN";
inline constexpr char COMPARE_OBJECTS[] = "COMPARE_OBJECTS";
inline constexpr char DISPOSE_TRASH[] = "DISPOSE_TRASH";
inline constexpr char GPSR_PREPARE_SKILL_SERVICE[] = "/gpsr_lifecycle/prepare_skill";
inline constexpr char GPSR_CLEANUP_SKILL_SERVICE[] = "/gpsr_lifecycle/cleanup_skill";
inline constexpr char GPSR_RECOVER_SKILL_SERVICE[] = "/gpsr_lifecycle/recover_skill";
inline constexpr char GPSR_GET_SKILL_STATE_SERVICE[] = "/gpsr_lifecycle/get_skill_state";

// Section: Preserve every shared blackboard key name.
namespace bb {
inline constexpr char ACTION_QUEUE[] = "action_queue";
inline constexpr char CURRENT_ACTION_INDEX[] = "current_action_index";
inline constexpr char CURRENT_ACTION[] = "current_action";
inline constexpr char LAST_SKILL_RESULT[] = "last_skill_result";
inline constexpr char SKILL_RESULTS[] = "skill_results";
inline constexpr char DISPATCH_OUTCOME[] = "dispatch_outcome";
inline constexpr char FAILURE_REASON[] = "failure_reason";
inline constexpr char RECOVERY_ATTEMPTS[] = "recovery_attempts";
inline constexpr char RECOVERY_HINT[] = "recovery_hint";
inline constexpr char RESULT_VARIABLES[] = "result_variables";
inline constexpr char COMPLETED_STEP_IDS[] = "completed_step_ids";
inline constexpr char CANCEL_REQUESTED[] = "cancel_requested";
inline constexpr char CURRENT_TASK_ID[] = "current_task_id";
inline constexpr char ACTIVE_LIFECYCLE_SKILL[] = "active_lifecycle_skill";
inline constexpr char ACTIVE_SKILL_GENERATION[] = "active_skill_generation";
inline constexpr char ACTIVE_SKILL_TARGET[] = "active_skill_target";
inline constexpr char START_CLEANUP_CONFIRMED[] = "start_cleanup_confirmed";
inline constexpr char TARGET_OBJECT[] = "target_object";
inline constexpr char TARGET_PERSON[] = "target_person";
inline constexpr char TARGET_POSE[] = "target_pose";
inline constexpr char HELD_OBJECT[] = "held_object";
inline constexpr char ENTITY_MEMORY[] = "entity_memory";
inline constexpr char COUNT_RESULT[] = "count_result";
inline constexpr char COUNT_OBJECTS_RESULT_JSON[] = "count_objects_result_json";
inline constexpr char COUNT_PEOPLE_RESULT_JSON[] = "count_people_result_json";
inline constexpr char OBJECT_QUANTITIES[] = "object_quantities";
inline constexpr char DETECTED_OBJECTS[] = "detected_objects";
inline constexpr char DETECTED_PEOPLE[] = "detected_people";
inline constexpr char SELECTED_OBJECT[] = "selected_object";
inline constexpr char EXTRACTED_PERSON_INFO[] = "extracted_person_info";
inline constexpr char DIALOGUE_ANSWER[] = "dialogue_answer";
inline constexpr char REPORT_TEXT[] = "report_text";
inline constexpr char INSTRUCTION_POINT[] = "instruction_point";
inline constexpr char PERSON_REF[] = "person_ref";
inline constexpr char PROFILE_REGISTERED[] = "profile_registered";
inline constexpr char IDENTIFICATION_MODE_USED[] = "identification_mode_used";
inline constexpr char GPSR_GUEST_MEMORY_ID[] = "gpsr_guest_memory_id";
inline constexpr char GPSR_GUEST_MEMORY_REVISION[] = "gpsr_guest_memory_revision";
}  // namespace bb

// Section: C++ equivalent of the Python ActionStepData dataclass.
struct ActionStepData {
  std::string step_id;
  std::string action_type;
  std::string task_id;
  std::string task_family;
  std::string target_object;
  std::string target_person;
  std::string target_location;
  std::string target_category;
  Json parameters{Json::object()};
  double timeout_sec{0.0};
  std::uint32_t max_retries{0};
  std::vector<std::string> requires_success;
  std::vector<std::string> requires_data;
  std::string result_variable;
};

// Section: C++ equivalent of the Python SkillResultData dataclass.
struct SkillResultData {
  std::string action_type;
  std::string outcome{FAILED};
  std::string message;
  Json payload{Json::object()};
  bool recoverable{false};
  bool fatal{false};
};

// Special function: Return the complete immutable YASMIN skill outcome set.
yasmin::Outcomes skill_outcomes();

// Special function: Parse JSON without allowing malformed transport data to terminate a node.
Json parse_json(const std::string &text, const Json &fallback = Json::object());

// Special function: Serialize JSON using the compact deterministic representation.
std::string dump_json(const Json &value);

// Special function: Convert the generated ROS ActionStep into the internal compatibility model.
ActionStepData action_from_msg(const robot_ai_interfaces::msg::ActionStep &message);

// Special function: Convert an internal deterministic result into its ROS message.
robot_ai_interfaces::msg::SkillResult result_to_msg(const SkillResultData &result);

// Special function: Convert a deterministic result to the JSON blackboard/status representation.
Json result_to_json(const SkillResultData &result);

// Special function: Construct a result while preserving Python default recovery flags.
SkillResultData make_skill_result(
    const std::string &action_type, const std::string &outcome,
    const std::string &message, const Json &payload = Json::object(),
    bool recoverable = false, bool fatal = false);

// Special function: Classify queue outcomes exactly like the Python helper.
bool is_recoverable(const std::string &outcome);

// Special function: Classify fatal queue outcomes exactly like the Python helper.
bool is_fatal(const std::string &outcome);

// Special function: Read a typed blackboard value or return its Python-equivalent default.
template <typename T>
T bb_get(const Blackboard::SharedPtr &blackboard, const std::string &key,
         const T &fallback) {
  if (!blackboard || !blackboard->contains(key)) {
    return fallback;
  }
  try {
    return blackboard->get<T>(key);
  } catch (const std::exception &) {
    return fallback;
  }
}

// Special function: Poll the queue-owned cancellation callback.
bool cancel_requested(const Blackboard::SharedPtr &blackboard);

// Special function: Convert the canonical JSON pose contract to PoseStamped.
std::optional<geometry_msgs::msg::PoseStamped> pose_from_json(const Json &raw);

// Special function: Serialize a PoseStamped without changing field names.
Json pose_to_json(const geometry_msgs::msg::PoseStamped &pose);

}  // namespace robot_yasmin_tasks

#endif  // ROBOT_YASMIN_TASKS__RUNTIME_HPP_
