// Section: Shared runtime contracts, serialization, outcomes, and pose conversion
// helpers.
#include "robot_yasmin_tasks/runtime.hpp"

#include <limits>

namespace robot_yasmin_tasks {

// Section: Shared outcome and JSON serialization helpers used by every GPSR component.
yasmin::Outcomes skill_outcomes() {
  // Section: Keep ordering aligned with outcomes.py for viewer and test parity.
  return {SUCCESS,       FAILED,      CANCELED,           TIMEOUT,
          INVALID_INPUT, NOT_FOUND,   NEED_CLARIFICATION, RECOVERABLE_FAILURE,
          FATAL_FAILURE, PERSON_LOST, SKILL_UNAVAILABLE,
          SKIPPED_DEPENDENCY_FAILED};
}

Json parse_json(const std::string &text, const Json &fallback) {
  // Section: Empty and malformed strings use the caller-provided Python-style default.
  if (text.empty()) {
    return fallback;
  }
  try {
    return Json::parse(text);
  } catch (const Json::exception &) {
    return fallback;
  }
}

std::string dump_json(const Json &value) {
  // Section: Compact JSON matches the transport-oriented Python serialization.
  return value.dump();
}

ActionStepData action_from_msg(const robot_ai_interfaces::msg::ActionStep &message) {
  // Section: Copy every public ActionStep field without reinterpretation.
  ActionStepData action;
  action.step_id = message.step_id;
  action.action_type = message.action_type;
  action.task_id = message.task_id;
  action.task_family = message.task_family;
  action.target_object = message.target_object;
  action.target_person = message.target_person;
  action.target_location = message.target_location;
  action.target_category = message.target_category;
  action.parameters = parse_json(message.parameters_json, Json::object());
  if (!action.parameters.is_object()) {
    action.parameters = Json::object();
  }
  action.timeout_sec = static_cast<double>(message.timeout_sec);
  action.max_retries = message.max_retries;
  action.requires_success = message.requires_success;
  action.requires_data = message.requires_data;
  action.result_variable = message.result_variable;
  return action;
}

robot_ai_interfaces::msg::SkillResult result_to_msg(const SkillResultData &result) {
  // Section: Preserve the generated SkillResult wire contract.
  robot_ai_interfaces::msg::SkillResult message;
  message.action_type = result.action_type;
  message.outcome = result.outcome;
  message.message = result.message;
  message.result_json = dump_json(result.payload);
  message.recoverable = result.recoverable;
  message.fatal = result.fatal;
  return message;
}

Json result_to_json(const SkillResultData &result) {
  // Section: Retain Python field names consumed by queue status and report skills.
  return Json{{"action_type", result.action_type}, {"outcome", result.outcome},
              {"message", result.message},         {"payload", result.payload},
              {"recoverable", result.recoverable}, {"fatal", result.fatal}};
}

SkillResultData make_skill_result(const std::string &action_type,
                                  const std::string &outcome,
                                  const std::string &message, const Json &payload,
                                  bool recoverable, bool fatal) {
  // Section: Every invalid/unavailable result has a stable code even when an older
  // caller supplied only a human-readable message.
  Json normalized = payload.is_object() ? payload : Json::object();
  if (!normalized.contains("failure_code") && outcome == INVALID_INPUT) {
    normalized.update(failure_payload(INVALID_INPUT_FAILURE, "validation"));
  } else if (!normalized.contains("failure_code") && outcome == SKILL_UNAVAILABLE) {
    normalized.update(failure_payload(SKILL_UNAVAILABLE_FAILURE, "skill_dispatch"));
  }
  return SkillResultData{action_type, outcome, message, normalized, recoverable, fatal};
}

Json failure_payload(const std::string &failure_code,
                     const std::string &failure_phase, const Json &details) {
  Json payload = details.is_object() ? details : Json::object();
  payload["failure_code"] = failure_code;
  payload["failure_phase"] = failure_phase;
  return payload;
}

bool is_recoverable(const std::string &outcome) {
  // Section: Preserve the exact recovery outcome set.
  static const std::set<std::string> values{
      FAILED, TIMEOUT, NOT_FOUND, NEED_CLARIFICATION, RECOVERABLE_FAILURE, PERSON_LOST};
  return values.count(outcome) != 0U;
}

bool is_fatal(const std::string &outcome) {
  // Section: Preserve the exact fatal outcome set.
  static const std::set<std::string> values{CANCELED, INVALID_INPUT, FATAL_FAILURE,
                                            SKILL_UNAVAILABLE};
  return values.count(outcome) != 0U;
}

std::string aggregate_queue_outcome(
    const std::vector<SkillResultData> &results) {
  // Section: An empty queue, like a queue containing only successful operations,
  // completed every executable request presented to it.
  if (results.empty()) {
    return SUCCESS;
  }
  const auto successful = std::count_if(
      results.begin(), results.end(),
      [](const SkillResultData &result) { return result.outcome == SUCCESS; });
  if (successful == static_cast<std::ptrdiff_t>(results.size())) {
    return SUCCESS;
  }
  // Section: Any useful success combined with a failed or skipped operation is a
  // normal partial completion; no successes is a completed-but-failed traversal.
  return successful > 0 ? PARTIAL_SUCCESS : FAILED;
}

bool cancel_requested(const Blackboard::SharedPtr &blackboard) {
  // Section: The queue stores cancellation as a callable, matching Python behavior.
  const auto callback = bb_get<std::function<bool()>>(blackboard, bb::CANCEL_REQUESTED,
                                                      std::function<bool()>{});
  return callback ? callback() : false;
}

// Section: Navigation pose conversion helpers that validate the shared map-frame
// schema.
std::optional<geometry_msgs::msg::PoseStamped> pose_from_json(const Json &raw) {
  // Section: Require the existing map-frame position/quaternion schema.
  if (!raw.is_object() || raw.value("frame_id", std::string{}) != "map" ||
      !raw.contains("position") || !raw.contains("orientation")) {
    return std::nullopt;
  }
  try {
    const auto &position = raw.at("position");
    const auto &orientation = raw.at("orientation");
    std::vector<double> values{
        position.at("x").get<double>(),    position.at("y").get<double>(),
        position.at("z").get<double>(),    orientation.at("x").get<double>(),
        orientation.at("y").get<double>(), orientation.at("z").get<double>(),
        orientation.at("w").get<double>()};
    if (!std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); })) {
      return std::nullopt;
    }
    const double norm = std::sqrt(values[3] * values[3] + values[4] * values[4] +
                                  values[5] * values[5] + values[6] * values[6]);
    if (norm <= 1.0e-9) {
      return std::nullopt;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = values[0];
    pose.pose.position.y = values[1];
    pose.pose.position.z = values[2];
    pose.pose.orientation.x = values[3] / norm;
    pose.pose.orientation.y = values[4] / norm;
    pose.pose.orientation.z = values[5] / norm;
    pose.pose.orientation.w = values[6] / norm;
    return pose;
  } catch (const Json::exception &) {
    return std::nullopt;
  }
}

Json pose_to_json(const geometry_msgs::msg::PoseStamped &pose) {
  // Section: Keep the JSON field topology consumed by navigation and history.
  return Json{{"frame_id", pose.header.frame_id},
              {"position",
               {{"x", pose.pose.position.x},
                {"y", pose.pose.position.y},
                {"z", pose.pose.position.z}}},
              {"orientation",
               {{"x", pose.pose.orientation.x},
                {"y", pose.pose.orientation.y},
                {"z", pose.pose.orientation.z},
                {"w", pose.pose.orientation.w}}}};
}

}  // namespace robot_yasmin_tasks
