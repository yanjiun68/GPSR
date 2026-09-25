#include "robot_yasmin_tasks/clients.hpp"

#include <action_msgs/msg/goal_status.hpp>
#include <algorithm>
#include <regex>
#include <thread>

namespace robot_yasmin_tasks {
namespace {

// Special function: Read one JSON value while preserving Python truthy-default
// behavior.
template <typename T>
T json_value(const Json &object, const std::string &key, const T &fallback) {
  if (!object.is_object() || !object.contains(key) || object.at(key).is_null()) {
    return fallback;
  }
  try {
    return object.at(key).get<T>();
  } catch (const Json::exception &) {
    return fallback;
  }
}

// Special function: Recreate the Python count-client filter JSON construction.
std::string build_filter_json(const ActionStepData &action) {
  // Section: An explicitly serialized filter remains authoritative.
  if (action.parameters.contains("filter_json")) {
    const auto &raw_filter = action.parameters.at("filter_json");
    if (raw_filter.is_string()) return raw_filter.get<std::string>();
    if (raw_filter.is_object()) return dump_json(raw_filter);
    return "{}";
  }
  // Section: Semantic compiler filters are an array; preserve it inside the service's
  // JSON envelope.
  Json payload = Json::object();
  const Json filters = action.parameters.value("filters", Json::array());
  if (filters.is_array()) {
    payload["filters"] = filters;
  } else if (filters.is_object()) {
    payload = filters;
  }
  // Section: Legacy count-control fields remain top-level and coexist with semantic ROI
  // constraints.
  for (const auto *key :
       {"min_confidence", "stale_timeout_sec", "enforce_location_filter"}) {
    if (action.parameters.contains(key)) {
      payload[key] = action.parameters.at(key);
    }
  }
  return dump_json(payload);
}

// Special function: Build one typed object-query envelope from generic ActionStep
// parameters.
Json build_find_query(const ActionStepData &action, const std::string &object_name,
                      const std::string &category) {
  // Section: Accept an explicit serialized query first, then the scheduler's native
  // query object.
  Json query = Json::object();
  if (action.parameters.contains("query_json")) {
    const auto raw = action.parameters.at("query_json");
    if (raw.is_string())
      query = parse_json(raw.get<std::string>(), Json::object());
    else if (raw.is_object())
      query = raw;
  }
  if ((!query.is_object() || query.empty()) && action.parameters.contains("query") &&
      action.parameters.at("query").is_object()) {
    query = action.parameters.at("query");
  }
  if (!query.is_object()) query = Json::object();

  // Section: Merge filters instead of replacing them so direct and planner callers
  // share one contract.
  Json filters = query.value("filters", Json::array());
  if (!filters.is_array()) filters = Json::array();
  const auto action_filters = action.parameters.value("filters", Json::array());
  if (action_filters.is_array()) {
    for (const auto &filter : action_filters) {
      if (filter.is_object() &&
          std::find(filters.begin(), filters.end(), filter) == filters.end()) {
        filters.push_back(filter);
      }
    }
  }
  // Section: Preserve historical extreme-mode payloads as an ordinary size selector.
  auto mode = json_value<std::string>(action.parameters, "mode", "");
  if (mode == "biggest") mode = "largest";
  if ((mode == "largest" || mode == "smallest") &&
      std::none_of(filters.begin(), filters.end(), [&mode](const Json &filter) {
        return filter.is_object() &&
               filter.value("feature", std::string{}) == "size_relation";
      })) {
    filters.push_back(
        Json{{"feature", "size_relation"}, {"operator", "equals"}, {"value", mode}});
  }
  query["filters"] = filters;
  query["target"] = Json{{"object_name", object_name}, {"category", category}};
  // Section: Default reporting can use deterministic bbox-derived image position on
  // every find result.
  if (!query.contains("requested_fields") || !query.at("requested_fields").is_array()) {
    query["requested_fields"] = Json::array({"image_position"});
  }
  return query;
}

// Special function: Wait in short intervals so queue cancellation remains observable.
template <typename FutureT>
bool wait_future(FutureT &future, double timeout_sec,
                 const Blackboard::SharedPtr &blackboard = nullptr) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(std::max(0.1, timeout_sec));
  while (std::chrono::steady_clock::now() < deadline) {
    if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
      return true;
    }
    if (blackboard && cancel_requested(blackboard)) {
      return false;
    }
  }
  return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

// Special function: Resolve safe dotted JSON paths used by deferred speech.
std::pair<std::string, std::vector<std::string>> render_template(
    const std::string &input, const Json &source) {
  static const std::regex expression(R"(\{([A-Za-z_][A-Za-z0-9_.]*)\})");
  std::string output;
  std::vector<std::string> missing;
  std::sregex_iterator cursor(input.begin(), input.end(), expression);
  const std::sregex_iterator end;
  std::size_t prior = 0;
  for (; cursor != end; ++cursor) {
    const auto &match = *cursor;
    output.append(
        input.substr(prior, static_cast<std::size_t>(match.position()) - prior));
    std::string path = match[1].str();
    std::string remainder = path.rfind("result.", 0) == 0 ? path.substr(7) : path;
    Json value = source;
    bool found = true;
    std::size_t begin = 0;
    while (begin <= remainder.size()) {
      const auto dot = remainder.find('.', begin);
      const auto part = remainder.substr(begin, dot - begin);
      if (!value.is_object() || !value.contains(part)) {
        found = false;
        break;
      }
      value = value.at(part);
      if (dot == std::string::npos) {
        break;
      }
      begin = dot + 1;
    }
    if (!found) {
      missing.push_back(path);
      output.append(match.str());
    } else if (value.is_string()) {
      output.append(value.get<std::string>());
    } else {
      output.append(value.dump());
    }
    prior = static_cast<std::size_t>(match.position() + match.length());
  }
  output.append(input.substr(prior));
  return {output, missing};
}

}  // namespace

SpeechClient::SpeechClient(const rclcpp::Node::SharedPtr &node, bool simulate,
                           std::string service_name)
    : node_(node), simulate_(simulate), service_name_(std::move(service_name)) {
  // Section: Simulation intentionally owns no ROS service handle.
  if (node_ && !simulate_) {
    client_ = node_->create_client<voice_interfaces::srv::SayText>(service_name_);
  }
}

SkillResultData SpeechClient::say(const Blackboard::SharedPtr &blackboard,
                                  const ActionStepData &action) {
  // Section: Select literal text before applying optional result substitution.
  std::string text = json_value<std::string>(action.parameters, "text", "");
  if (text.empty()) {
    text = json_value<std::string>(action.parameters, "text_template", "");
  }
  if (text.empty()) {
    return make_skill_result(action.action_type, FAILED, "Speech text missing.",
                             Json::object(), true);
  }
  const auto substitution =
      json_value<std::string>(action.parameters, "substitution", "");
  if (!substitution.empty()) {
    Json source = Json::object();
    if (action.parameters.contains("substitution_data") &&
        action.parameters.at("substitution_data").is_object()) {
      source = action.parameters.at("substitution_data");
    } else {
      const auto variables =
          bb_get<Json>(blackboard, bb::RESULT_VARIABLES, Json::object());
      if (variables.is_object() && variables.contains(substitution)) {
        source = variables.at(substitution);
      } else {
        source = bb_get<Json>(blackboard, substitution, Json::object());
      }
    }
    auto rendered = render_template(text, source);
    text = std::move(rendered.first);
    if (!rendered.second.empty()) {
      return make_skill_result(action.action_type, FAILED,
                               "Speech substitution fields are missing: " +
                                   Json(rendered.second).dump() + ".",
                               Json{{"missing_fields", rendered.second}}, true);
    }
  }
  if (simulate_) {
    return make_skill_result(action.action_type, SUCCESS, "Simulated speech completed.",
                             Json{{"text", text}});
  }
  if (!client_) {
    return make_skill_result(action.action_type, FAILED,
                             "TTS client unavailable: " + service_name_, Json::object(),
                             true);
  }
  if (!client_->wait_for_service(std::chrono::seconds(3))) {
    return make_skill_result(action.action_type, FAILED,
                             "TTS service unavailable: " + service_name_,
                             Json::object(), true);
  }
  auto request = std::make_shared<voice_interfaces::srv::SayText::Request>();
  request->text = text;
  auto future = client_->async_send_request(request);
  const double timeout = action.timeout_sec != 0.0 ? action.timeout_sec : 8.0;
  if (!wait_future(future, timeout)) {
    return make_skill_result(action.action_type, FAILED, "TTS response timed out.",
                             Json::object(), true);
  }
  const auto response = future.get();
  if (response && response->success) {
    return make_skill_result(action.action_type, SUCCESS, response->message,
                             Json{{"text", text}});
  }
  return make_skill_result(action.action_type, FAILED,
                           response ? response->message : "TTS failed.", Json::object(),
                           true);
}

DialogueClient::DialogueClient(const rclcpp::Node::SharedPtr &node,
                               const std::shared_ptr<SpeechClient> &speech,
                               bool simulate)
    : node_(node), speech_(speech), simulate_(simulate) {
  // Section: Match the Python depth-10 voice topic endpoints.
  if (node_ && !simulate_) {
    start_pub_ =
        node_->create_publisher<std_msgs::msg::Float32>("/voice/stt/start", 10);
    transcript_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/voice/stt/transcript", 10,
        std::bind(&DialogueClient::transcript_callback, this, std::placeholders::_1));
  }
}

void DialogueClient::transcript_callback(
    const std_msgs::msg::String::SharedPtr message) {
  // Section: Cache the latest transcript under the same single-answer policy.
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = message ? message->data : "";
  ++sequence_;
  condition_.notify_all();
}

SkillResultData DialogueClient::ask_and_listen(const Blackboard::SharedPtr &blackboard,
                                               const ActionStepData &action) {
  // Section: Preserve explicit simulation timeout and answer behavior.
  if (json_value<bool>(action.parameters, "simulate_timeout", false)) {
    return make_skill_result(action.action_type, TIMEOUT, "Dialogue timed out.",
                             Json::object(), true);
  }
  if (simulate_ || !start_pub_) {
    const auto answer =
        json_value<std::string>(action.parameters, "simulated_answer", "");
    if (answer.empty()) {
      return make_skill_result(action.action_type, NEED_CLARIFICATION,
                               "No clear answer.", Json::object(), true);
    }
    blackboard->set<std::string>(bb::DIALOGUE_ANSWER, answer);
    return make_skill_result(action.action_type, SUCCESS, "Answer received.",
                             Json{{"answer", answer}});
  }
  const auto question =
      json_value<std::string>(action.parameters, "question",
                              json_value<std::string>(action.parameters, "text", ""));
  if (!question.empty()) {
    auto speech_action = action;
    speech_action.parameters["text"] = question;
    const auto speech_result = speech_->say(blackboard, speech_action);
    if (speech_result.outcome != SUCCESS) {
      return speech_result;
    }
  }
  std::uint64_t initial_sequence;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.clear();
    initial_sequence = sequence_;
  }
  std_msgs::msg::Float32 trigger;
  trigger.data =
      static_cast<float>(json_value<double>(action.parameters, "record_sec", 4.0));
  start_pub_->publish(trigger);
  const double listen_timeout =
      json_value<double>(action.parameters, "listen_timeout_sec", trigger.data + 8.0);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(listen_timeout);
  std::unique_lock<std::mutex> lock(mutex_);
  while (std::chrono::steady_clock::now() < deadline) {
    if (cancel_requested(blackboard)) {
      return make_skill_result(action.action_type, CANCELED, "Dialogue canceled.",
                               Json::object(), false, true);
    }
    condition_.wait_for(lock, std::chrono::milliseconds(50));
    if (sequence_ != initial_sequence) {
      const auto answer = latest_;
      lock.unlock();
      blackboard->set<std::string>(bb::DIALOGUE_ANSWER, answer);
      return make_skill_result(action.action_type, SUCCESS,
                               "Local STT answer received.", Json{{"answer", answer}});
    }
  }
  return make_skill_result(action.action_type, NEED_CLARIFICATION,
                           "No STT transcript received.", Json::object(), true);
}

FixedInformationClient::FixedInformationClient(
    const rclcpp::Node::SharedPtr &node, const std::shared_ptr<SpeechClient> &speech,
    bool simulate)
    : node_(node), speech_(speech), simulate_(simulate) {
  // Section: The provider uses JSON envelopes over paired depth-10 string topics.
  if (node_ && !simulate_) {
    query_pub_ =
        node_->create_publisher<std_msgs::msg::String>("/fixed_information/query", 10);
    answer_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/fixed_information/answer", 10,
        std::bind(&FixedInformationClient::answer_callback, this,
                  std::placeholders::_1));
  }
}

void FixedInformationClient::answer_callback(
    const std_msgs::msg::String::SharedPtr message) {
  // Section: Malformed or unrelated envelopes cannot complete the active state.
  if (!message) return;
  const Json payload = parse_json(message->data, Json::object());
  if (!payload.is_object()) return;
  const auto request_id = payload.value("request_id", std::string{});
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_request_id_.empty() || request_id != pending_request_id_) return;
  latest_answer_ = payload;
  ++sequence_;
  condition_.notify_all();
}

SkillResultData FixedInformationClient::answer(const Blackboard::SharedPtr &blackboard,
                                               const ActionStepData &action) {
  // Section: Preserve the complete scheduler question for provider-owned alias
  // matching.
  const auto question =
      json_value<std::string>(action.parameters, "question",
                              json_value<std::string>(action.parameters, "text", ""));
  if (question.empty()) {
    return make_skill_result(action.action_type, INVALID_INPUT,
                             "Fixed-information question missing.", Json::object(),
                             false, true);
  }
  // Section: Simulation provides a deterministic state path without ROS topic
  // dependencies.
  if (simulate_) {
    const auto simulated_answer =
        json_value<std::string>(action.parameters, "simulated_answer", "");
    if (simulated_answer.empty()) {
      return make_skill_result(action.action_type, NOT_FOUND,
                               "Simulated fixed-information answer missing.",
                               Json{{"question", question}}, true);
    }
    blackboard->set<std::string>(bb::REPORT_TEXT, simulated_answer);
    return make_skill_result(
        action.action_type, SUCCESS, "Fixed-information answer resolved.",
        Json{{"question", question}, {"answer_text", simulated_answer}});
  }
  if (!query_pub_ || !answer_sub_) {
    return make_skill_result(action.action_type, FAILED,
                             "Fixed-information topic client unavailable.",
                             Json::object(), true);
  }

  // Section: A unique request identifier prevents another dialogue caller's result from
  // leaking in.
  const auto request_id =
      "gpsr-fixed-" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  std::uint64_t initial_sequence;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_request_id_ = request_id;
    latest_answer_ = Json::object();
    initial_sequence = sequence_;
  }
  std_msgs::msg::String request;
  request.data = Json{{"request_id", request_id}, {"text", question}}.dump();
  query_pub_->publish(request);

  // Section: Wait briefly for the local deterministic provider while honoring queue
  // cancellation.
  const double provider_timeout =
      json_value<double>(action.parameters, "provider_timeout_sec", 2.0);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(std::max(0.1, provider_timeout));
  Json payload = Json::object();
  std::unique_lock<std::mutex> lock(mutex_);
  while (std::chrono::steady_clock::now() < deadline) {
    if (cancel_requested(blackboard)) {
      pending_request_id_.clear();
      return make_skill_result(action.action_type, CANCELED,
                               "Fixed-information query canceled.", Json::object(),
                               false, true);
    }
    condition_.wait_for(lock, std::chrono::milliseconds(50));
    if (sequence_ != initial_sequence) {
      payload = latest_answer_;
      pending_request_id_.clear();
      break;
    }
  }
  pending_request_id_.clear();
  lock.unlock();
  if (payload.empty()) {
    return make_skill_result(action.action_type, TIMEOUT,
                             "Fixed-information provider timed out.",
                             Json{{"question", question}}, true);
  }

  // Section: Preserve provider status semantics instead of converting misses into
  // speech guesses.
  const auto status = payload.value("status", std::string{});
  const auto answer_text = payload.value("answer_text", std::string{});
  if (status == "AMBIGUOUS") {
    return make_skill_result(action.action_type, NEED_CLARIFICATION,
                             "Fixed-information question is ambiguous.", payload, true);
  }
  if (status == "NO_MATCH") {
    return make_skill_result(action.action_type, NOT_FOUND,
                             "Fixed-information provider has no matching answer.",
                             payload, true);
  }
  if (status != "FOUND" || answer_text.empty()) {
    return make_skill_result(action.action_type, FAILED,
                             "Fixed-information answer is not configured.", payload,
                             true);
  }

  // Section: Speak exactly the provider-owned answer, then expose it on the shared
  // blackboard.
  auto speech_action = action;
  speech_action.parameters = Json{{"text", answer_text}};
  const auto speech_result = speech_->say(blackboard, speech_action);
  if (speech_result.outcome != SUCCESS) return speech_result;
  blackboard->set<std::string>(bb::REPORT_TEXT, answer_text);
  payload["question"] = question;
  return make_skill_result(action.action_type, SUCCESS,
                           "Fixed-information answer spoken.", payload);
}

Nav2Client::Nav2Client(const rclcpp::Node::SharedPtr &node, bool simulate,
                       std::string named_action, std::string pose_action,
                       std::string spin_action)
    : node_(node),
      simulate_(simulate),
      named_action_(std::move(named_action)),
      pose_action_(std::move(pose_action)),
      spin_action_(std::move(spin_action)) {
  // Section: The executor owns all action clients for their complete lifetime.
  if (node_ && !simulate_) {
    named_client_ =
        rclcpp_action::create_client<robot_ai_interfaces::action::NavigateNamedPose>(
            node_, named_action_);
    pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        node_, pose_action_);
    spin_client_ =
        rclcpp_action::create_client<nav2_msgs::action::Spin>(node_, spin_action_);
  }
}

SkillResultData Nav2Client::navigate_to_pose(const Blackboard::SharedPtr &blackboard,
                                             const ActionStepData &action) {
  // Section: Enforce the existing mutually exclusive named/raw target contract.
  std::string location = action.target_location;
  if (location.empty()) {
    location = json_value<std::string>(action.parameters, "target_location", "");
  }
  if (location.empty()) {
    location = json_value<std::string>(action.parameters, "location", "");
  }
  const bool has_pose = action.parameters.contains("target_pose") &&
                        !action.parameters.at("target_pose").is_null();
  if (!location.empty() && has_pose) {
    return make_skill_result(
        action.action_type, FAILED,
        "NAVIGATE requires exactly one of target_location or target_pose.",
        Json::object(), false, true);
  }
  const double timeout =
      action.timeout_sec != 0.0
          ? action.timeout_sec
          : json_value<double>(action.parameters, "timeout_sec", 60.0);
  if (has_pose) {
    const auto pose = pose_from_json(action.parameters.at("target_pose"));
    if (!pose) {
      return make_skill_result(action.action_type, FAILED,
                               "Raw navigation target_pose is malformed.",
                               Json::object(), false, true);
    }
    if (simulate_) {
      return make_skill_result(action.action_type, SUCCESS,
                               "Simulated raw-pose navigation completed.",
                               Json{{"target", pose_to_json(*pose)},
                                    {"simulated", true},
                                    {"target_type", "pose"}});
    }
    if (!pose_client_ || !pose_client_->wait_for_action_server(
                             std::chrono::duration<double>(std::min(5.0, timeout)))) {
      return make_skill_result(action.action_type, FAILED,
                               "NavigateToPose action unavailable: " + pose_action_,
                               Json{{"target", pose_to_json(*pose)}}, true);
    }
    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose = *pose;
    goal.pose.header.stamp = node_->now();
    auto goal_future = pose_client_->async_send_goal(goal);
    if (!wait_future(goal_future, std::min(5.0, timeout), blackboard)) {
      return make_skill_result(
          action.action_type, cancel_requested(blackboard) ? CANCELED : TIMEOUT,
          "Navigation goal acceptance was canceled or timed out.", Json::object(),
          !cancel_requested(blackboard), cancel_requested(blackboard));
    }
    auto handle = goal_future.get();
    if (!handle) {
      return make_skill_result(action.action_type, FAILED,
                               "NavigateToPose goal was rejected.", Json::object(),
                               true);
    }
    auto result_future = pose_client_->async_get_result(handle);
    if (!wait_future(result_future, timeout, blackboard)) {
      pose_client_->async_cancel_goal(handle);
      const bool canceled = cancel_requested(blackboard);
      return make_skill_result(
          action.action_type, canceled ? CANCELED : TIMEOUT,
          canceled ? "Navigation canceled." : "Navigation result timed out.",
          Json::object(), !canceled, canceled);
    }
    const auto wrapped = result_future.get();
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return make_skill_result(
          action.action_type, SUCCESS, "Raw-pose navigation completed.",
          Json{{"target", pose_to_json(*pose)}, {"target_type", "pose"}});
    }
    return make_skill_result(
        action.action_type, FAILED, "Raw-pose navigation failed.",
        Json{{"target", pose_to_json(*pose)}, {"target_type", "pose"}}, true);
  }
  if (location.empty()) {
    return make_skill_result(action.action_type, FAILED, "Navigation target missing.",
                             Json::object(), true);
  }
  if (simulate_) {
    return make_skill_result(
        action.action_type, SUCCESS, "Simulated named navigation completed.",
        Json{{"target", location}, {"simulated", true}, {"target_type", "named"}});
  }
  if (!named_client_ || !named_client_->wait_for_action_server(
                            std::chrono::duration<double>(std::min(5.0, timeout)))) {
    return make_skill_result(action.action_type, FAILED,
                             "NavigateNamedPose action unavailable: " + named_action_,
                             Json{{"target", location}}, true);
  }
  robot_ai_interfaces::action::NavigateNamedPose::Goal goal;
  goal.target_name = location;
  auto goal_future = named_client_->async_send_goal(goal);
  if (!wait_future(goal_future, std::min(5.0, timeout), blackboard)) {
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(action.action_type, canceled ? CANCELED : TIMEOUT,
                             "Navigation goal acceptance was canceled or timed out.",
                             Json::object(), !canceled, canceled);
  }
  auto handle = goal_future.get();
  if (!handle) {
    return make_skill_result(action.action_type, FAILED,
                             "NavigateNamedPose goal was rejected.", Json::object(),
                             true);
  }
  auto result_future = named_client_->async_get_result(handle);
  if (!wait_future(result_future, timeout, blackboard)) {
    named_client_->async_cancel_goal(handle);
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(
        action.action_type, canceled ? CANCELED : TIMEOUT,
        canceled ? "Navigation canceled." : "Navigation result timed out.",
        Json::object(), !canceled, canceled);
  }
  const auto wrapped = result_future.get();
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
      wrapped.result->success) {
    return make_skill_result(action.action_type, SUCCESS, wrapped.result->message,
                             Json{{"target", location}, {"target_type", "named"}});
  }
  return make_skill_result(
      action.action_type, FAILED,
      wrapped.result ? wrapped.result->message : "Named navigation failed.",
      Json{{"target", location}, {"target_type", "named"}}, true);
}

SkillResultData Nav2Client::spin(const Blackboard::SharedPtr &blackboard,
                                 const ActionStepData &action, double yaw_radians,
                                 double timeout_sec) {
  // Section: Preserve the Nav2 Spin action and its relative yaw semantics.
  if (simulate_) {
    return make_skill_result(action.action_type, SUCCESS, "Simulated spin completed.",
                             Json{{"yaw", yaw_radians}});
  }
  if (!spin_client_ || !spin_client_->wait_for_action_server(
                           std::chrono::duration<double>(std::min(5.0, timeout_sec)))) {
    return make_skill_result(action.action_type, FAILED,
                             "Nav2 Spin action unavailable: " + spin_action_,
                             Json::object(), true);
  }
  nav2_msgs::action::Spin::Goal goal;
  goal.target_yaw = static_cast<float>(yaw_radians);
  goal.time_allowance = rclcpp::Duration::from_seconds(timeout_sec);
  auto goal_future = spin_client_->async_send_goal(goal);
  if (!wait_future(goal_future, std::min(5.0, timeout_sec), blackboard)) {
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(action.action_type, canceled ? CANCELED : TIMEOUT,
                             "Spin goal acceptance timed out.", Json::object(),
                             !canceled, canceled);
  }
  auto handle = goal_future.get();
  if (!handle) {
    return make_skill_result(action.action_type, FAILED, "Spin goal was rejected.",
                             Json::object(), true);
  }
  auto result_future = spin_client_->async_get_result(handle);
  if (!wait_future(result_future, timeout_sec, blackboard)) {
    spin_client_->async_cancel_goal(handle);
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(action.action_type, canceled ? CANCELED : TIMEOUT,
                             "Spin action timed out.", Json::object(), !canceled,
                             canceled);
  }
  return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED
             ? make_skill_result(action.action_type, SUCCESS, "Spin completed.",
                                 Json{{"yaw", yaw_radians}})
             : make_skill_result(action.action_type, FAILED, "Spin failed.",
                                 Json{{"yaw", yaw_radians}}, true);
}

PerceptionClient::PerceptionClient(const rclcpp::Node::SharedPtr &node, bool simulate,
                                   std::string object_action, std::string people_action)
    : node_(node),
      simulate_(simulate),
      object_action_(std::move(object_action)),
      people_action_(std::move(people_action)) {
  // Section: Keep both action clients persistent on the executor node.
  if (node_ && !simulate_) {
    object_client_ =
        rclcpp_action::create_client<robot_ai_interfaces::action::CountObjects>(
            node_, object_action_);
    people_client_ =
        rclcpp_action::create_client<robot_ai_interfaces::action::CountPeople>(
            node_, people_action_);
  }
}

SkillResultData PerceptionClient::count_objects(const Blackboard::SharedPtr &blackboard,
                                                const ActionStepData &action) {
  // Section: Preserve deterministic simulation used by camera-free tests.
  if (simulate_ || !object_client_) {
    const int count = json_value<int>(action.parameters, "simulated_count", 0);
    const Json quantities =
        action.parameters.value("simulated_object_quantities", Json::object());
    const Json detected =
        action.parameters.value("simulated_detected_objects", Json::array());
    blackboard->set<int>(bb::COUNT_RESULT, count);
    blackboard->set<Json>(bb::OBJECT_QUANTITIES, quantities);
    blackboard->set<Json>(bb::DETECTED_OBJECTS, detected);
    return make_skill_result(action.action_type, SUCCESS, "Objects counted.",
                             Json{{"count", count},
                                  {"object_quantities", quantities},
                                  {"detected_objects", detected}});
  }
  const double timeout = action.timeout_sec != 0.0 ? action.timeout_sec : 5.0;
  if (!object_client_->wait_for_action_server(std::chrono::duration<double>(timeout))) {
    return make_skill_result(action.action_type, FAILED,
                             "CountObjects action unavailable: " + object_action_,
                             Json::object(), true);
  }
  robot_ai_interfaces::action::CountObjects::Goal goal;
  goal.category =
      !action.target_category.empty()
          ? action.target_category
          : (!action.target_object.empty()
                 ? action.target_object
                 : json_value<std::string>(action.parameters, "category", ""));
  goal.location = !action.target_location.empty()
                      ? action.target_location
                      : json_value<std::string>(action.parameters, "location", "");
  goal.filter_json = build_filter_json(action);
  goal.timeout_sec = static_cast<float>(timeout);
  auto goal_future = object_client_->async_send_goal(goal);
  if (!wait_future(goal_future, timeout, blackboard) || !goal_future.get()) {
    return make_skill_result(action.action_type, FAILED,
                             "CountObjects action goal was rejected or timed out.",
                             Json::object(), true);
  }
  auto handle = goal_future.get();
  auto result_future = object_client_->async_get_result(handle);
  if (!wait_future(result_future, timeout, blackboard)) {
    object_client_->async_cancel_goal(handle);
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(action.action_type, canceled ? CANCELED : FAILED,
                             canceled ? "CountObjects action canceled."
                                      : "CountObjects action result timed out.",
                             Json::object(), !canceled, canceled);
  }
  const auto wrapped = result_future.get();
  const auto result = wrapped.result;
  const Json payload =
      result ? parse_json(result->result_json, Json::object()) : Json::object();
  const int count = result ? result->count : 0;
  blackboard->set<int>(bb::COUNT_RESULT, count);
  blackboard->set<std::string>(bb::COUNT_OBJECTS_RESULT_JSON,
                               result ? result->result_json : "{}");
  blackboard->set<Json>(bb::OBJECT_QUANTITIES,
                        payload.value("object_quantities", Json::object()));
  blackboard->set<Json>(bb::DETECTED_OBJECTS,
                        payload.value("all_detected_objects", Json::array()));
  Json skill_payload{
      {"count", count},
      {"object_quantities", payload.value("object_quantities", Json::object())},
      {"detected_objects", payload.value("all_detected_objects", Json::array())},
      {"result_json", payload}};
  return result && result->success
             ? make_skill_result(action.action_type, SUCCESS, result->message,
                                 skill_payload)
             : make_skill_result(action.action_type, FAILED,
                                 result ? result->message : "CountObjects failed.",
                                 skill_payload, true);
}

SkillResultData PerceptionClient::count_people(const Blackboard::SharedPtr &blackboard,
                                               const ActionStepData &action) {
  // Section: Preserve deterministic people-count simulation.
  if (simulate_ || !people_client_) {
    const int count = json_value<int>(action.parameters, "simulated_count", 0);
    const Json detected =
        action.parameters.value("simulated_detected_people", Json::array());
    const Json result_json =
        action.parameters.value("simulated_result_json", Json::object());
    blackboard->set<int>(bb::COUNT_RESULT, count);
    blackboard->set<std::string>(bb::COUNT_PEOPLE_RESULT_JSON, dump_json(result_json));
    blackboard->set<Json>(bb::DETECTED_PEOPLE, detected);
    return make_skill_result(action.action_type, SUCCESS, "People counted.",
                             Json{{"count", count},
                                  {"detected_people", detected},
                                  {"result_json", result_json}});
  }
  const double timeout = action.timeout_sec != 0.0 ? action.timeout_sec : 5.0;
  if (!people_client_->wait_for_action_server(std::chrono::duration<double>(timeout))) {
    return make_skill_result(action.action_type, FAILED,
                             "CountPeople action unavailable: " + people_action_,
                             Json::object(), true);
  }
  robot_ai_interfaces::action::CountPeople::Goal goal;
  goal.location = !action.target_location.empty()
                      ? action.target_location
                      : json_value<std::string>(action.parameters, "location", "");
  goal.filter_json = build_filter_json(action);
  goal.timeout_sec = static_cast<float>(timeout);
  auto goal_future = people_client_->async_send_goal(goal);
  if (!wait_future(goal_future, timeout, blackboard)) {
    return make_skill_result(action.action_type, FAILED,
                             "CountPeople action goal was rejected or timed out.",
                             Json::object(), true);
  }
  auto handle = goal_future.get();
  if (!handle) {
    return make_skill_result(action.action_type, FAILED,
                             "CountPeople action goal was rejected or timed out.",
                             Json::object(), true);
  }
  auto result_future = people_client_->async_get_result(handle);
  if (!wait_future(result_future, timeout, blackboard)) {
    people_client_->async_cancel_goal(handle);
    return make_skill_result(action.action_type, FAILED,
                             "CountPeople action result timed out.", Json::object(),
                             true);
  }
  const auto result = result_future.get().result;
  const Json payload =
      result ? parse_json(result->result_json, Json::object()) : Json::object();
  const Json detected = payload.value("human_entities", Json::array());
  const int count = result ? result->count : 0;
  blackboard->set<int>(bb::COUNT_RESULT, count);
  blackboard->set<std::string>(bb::COUNT_PEOPLE_RESULT_JSON,
                               result ? result->result_json : "{}");
  blackboard->set<Json>(bb::DETECTED_PEOPLE, detected);
  Json skill_payload{
      {"count", count}, {"detected_people", detected}, {"result_json", payload}};
  return result && result->success
             ? make_skill_result(action.action_type, SUCCESS, result->message,
                                 skill_payload)
             : make_skill_result(action.action_type, FAILED,
                                 result ? result->message : "CountPeople failed.",
                                 skill_payload, true);
}

SkillResultData PerceptionClient::compare_objects(
    const Blackboard::SharedPtr &blackboard, const ActionStepData &action) {
  // Section: Preserve local selected-object result behavior.
  const auto selected = json_value<std::string>(
      action.parameters, "selected_object",
      action.target_object.empty() ? "selected_object" : action.target_object);
  blackboard->set<std::string>(bb::SELECTED_OBJECT, selected);
  blackboard->set<std::string>(bb::TARGET_OBJECT, selected);
  return make_skill_result(action.action_type, SUCCESS, "Objects compared.",
                           Json{{"selected_object", selected}});
}

SkillResultData PerceptionClient::extract_person_info(
    const Blackboard::SharedPtr &blackboard, const ActionStepData &action) {
  // Section: Preserve the legacy local extraction fallback exactly.
  const auto field = json_value<std::string>(action.parameters, "field", "name");
  const auto value =
      json_value<std::string>(action.parameters, "value", action.target_person);
  const Json info{{field, value}};
  blackboard->set<Json>(bb::EXTRACTED_PERSON_INFO, info);
  return make_skill_result(action.action_type, SUCCESS, "Person info extracted.", info);
}

FollowPersonClient::FollowPersonClient(const rclcpp::Node::SharedPtr &node,
                                       std::string action_name)
    : node_(node), action_name_(std::move(action_name)) {
  // Section: Keep the production follow action endpoint unchanged.
  client_ = rclcpp_action::create_client<putrabot_interfaces::action::PersonFollow>(
      node_, action_name_);
}

SkillResultData FollowPersonClient::follow(const Blackboard::SharedPtr &blackboard,
                                           const ActionStepData &action) {
  // Section: Preserve goal defaults and bounded action behavior.
  const double timeout =
      action.timeout_sec != 0.0
          ? action.timeout_sec
          : json_value<double>(action.parameters, "timeout_sec", 120.0);
  if (!client_->wait_for_action_server(
          std::chrono::duration<double>(std::min(timeout, 10.0)))) {
    return make_skill_result(action.action_type, FAILED,
                             "FollowPerson action unavailable: " + action_name_,
                             Json::object(), true);
  }
  putrabot_interfaces::action::PersonFollow::Goal goal;
  goal.auto_select_if_needed =
      json_value<bool>(action.parameters, "auto_select_if_needed", true);
  goal.follow_distance_m = static_cast<float>(
      json_value<double>(action.parameters, "follow_distance_m", 1.2));
  goal.face_hold_sec =
      static_cast<float>(json_value<double>(action.parameters, "face_hold_sec", 5.0));
  Json feedback = Json::object();
  rclcpp_action::Client<putrabot_interfaces::action::PersonFollow>::SendGoalOptions
      options;
  options.feedback_callback = [&feedback](auto, const auto feedback_message) {
    // Section: Cache only the latest standard feedback fields.
    feedback = Json{{"selected_track_id", feedback_message->selected_track_id},
                    {"target_visible", feedback_message->target_visible},
                    {"target_age_sec", feedback_message->target_age_sec},
                    {"facing_robot", feedback_message->facing_robot},
                    {"status", feedback_message->stop_reason}};
  };
  auto goal_future = client_->async_send_goal(goal, options);
  if (!wait_future(goal_future, std::min(timeout, 10.0), blackboard)) {
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(action.action_type, canceled ? CANCELED : TIMEOUT,
                             "FollowPerson goal acceptance was canceled or timed out.",
                             feedback, !canceled, canceled);
  }
  auto handle = goal_future.get();
  if (!handle) {
    return make_skill_result(action.action_type, FAILED,
                             "FollowPerson goal was rejected.", feedback, true);
  }
  auto result_future = client_->async_get_result(handle);
  if (!wait_future(result_future, timeout, blackboard)) {
    client_->async_cancel_goal(handle);
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(
        action.action_type, canceled ? CANCELED : TIMEOUT,
        canceled ? "FollowPerson canceled by the GPSR queue."
                 : "FollowPerson exceeded its ActionStep timeout and was canceled.",
        feedback, !canceled, canceled);
  }
  const auto wrapped = result_future.get();
  const auto result = wrapped.result;
  const std::string message = result ? result->message : "FollowPerson failed.";
  std::string outcome = FAILED;
  if (result && result->success) {
    outcome = SUCCESS;
  } else if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
    outcome = CANCELED;
  } else {
    std::string reason = message;
    std::transform(reason.begin(), reason.end(), reason.begin(), ::tolower);
    if (reason.rfind("person_lost:", 0) == 0) {
      outcome = PERSON_LOST;
    } else if (reason.rfind("person_not_found:", 0) == 0 ||
               reason.rfind("person_not_selected:", 0) == 0) {
      outcome = NOT_FOUND;
    }
  }
  feedback["target_person"] = action.target_person;
  feedback["follow_distance_m"] = goal.follow_distance_m;
  feedback["face_hold_sec"] = goal.face_hold_sec;
  return make_skill_result(action.action_type, outcome, message, feedback,
                           outcome == FAILED || outcome == NOT_FOUND ||
                               outcome == PERSON_LOST || outcome == TIMEOUT,
                           outcome == CANCELED);
}

RemoteSkillClient::RemoteSkillClient(const rclcpp::Node::SharedPtr &node)
    : node_(node) {
  // Section: Keep lifecycle-owned action endpoint names unchanged.
  find_object_client_ =
      rclcpp_action::create_client<robot_ai_interfaces::action::FindObject>(
          node_, "/gpsr/find_object");
  find_attribute_client_ =
      rclcpp_action::create_client<robot_ai_interfaces::action::FindPersonByAttribute>(
          node_, "/gpsr/find_person_by_attribute");
  extract_client_ =
      rclcpp_action::create_client<robot_ai_interfaces::action::ExtractGuestInfo>(
          node_, "/gpsr/extract_guest_info");
}

SkillResultData RemoteSkillClient::find_object(const Blackboard::SharedPtr &blackboard,
                                               const ActionStepData &action) {
  // Section: Validate the public target and search-scope contract before goal
  // submission.
  const auto object_name =
      !action.target_object.empty()
          ? action.target_object
          : json_value<std::string>(action.parameters, "object_name", "");
  const auto category =
      !action.target_category.empty()
          ? action.target_category
          : json_value<std::string>(action.parameters, "category", "");
  if (object_name.empty() && category.empty()) {
    return make_skill_result(action.action_type, INVALID_INPUT,
                             "FIND_OBJECT requires object_name or category.",
                             Json::object(), false, true);
  }
  std::string location =
      !action.target_location.empty()
          ? action.target_location
          : json_value<std::string>(action.parameters, "location", "");
  if (location.empty()) {
    location = json_value<std::string>(action.parameters, "target_location", "");
  }
  std::string scope =
      json_value<std::string>(action.parameters, "search_scope",
                              location.empty() ? "GLOBAL" : "STRICT_LOCATION");
  std::transform(scope.begin(), scope.end(), scope.begin(), ::toupper);
  if ((scope != "GLOBAL" && scope != "STRICT_LOCATION") ||
      (scope == "STRICT_LOCATION" && location.empty())) {
    return make_skill_result(
        action.action_type, INVALID_INPUT,
        "FIND_OBJECT search_scope/location combination is invalid.", Json::object(),
        false, true);
  }
  const double timeout =
      action.timeout_sec != 0.0
          ? action.timeout_sec
          : json_value<double>(action.parameters, "timeout_sec", 300.0);
  if (!find_object_client_->wait_for_action_server(std::chrono::seconds(5))) {
    return make_skill_result(
        action.action_type, RECOVERABLE_FAILURE,
        "FindObject action server /gpsr/find_object is unavailable.", Json::object(),
        true);
  }
  robot_ai_interfaces::action::FindObject::Goal goal;
  goal.object_name = object_name;
  goal.category = category;
  goal.location = location;
  goal.search_scope = scope;
  // Section: A bare FIND_OBJECT is a visual inspection; pickup macros explicitly opt
  // into approach.
  goal.approach_target = json_value<bool>(action.parameters, "approach_target", false);
  goal.stand_off_distance = static_cast<float>(
      json_value<double>(action.parameters, "stand_off_distance", 0.8));
  goal.timeout_sec = static_cast<float>(timeout);
  goal.preferred_viewpoint =
      json_value<std::string>(action.parameters, "preferred_viewpoint", "");
  // Section: One action goal carries all characteristic and relation constraints to the
  // Qwen detector.
  goal.query_json = dump_json(build_find_query(action, object_name, category));
  auto goal_future = find_object_client_->async_send_goal(goal);
  if (!wait_future(goal_future, 5.0, blackboard)) {
    return make_skill_result(action.action_type, RECOVERABLE_FAILURE,
                             "FindObject goal was rejected or timed out.",
                             Json::object(), true);
  }
  auto handle = goal_future.get();
  if (!handle) {
    return make_skill_result(action.action_type, RECOVERABLE_FAILURE,
                             "FindObject goal was rejected or timed out.",
                             Json::object(), true);
  }
  auto result_future = find_object_client_->async_get_result(handle);
  if (!wait_future(result_future, timeout + 5.0, blackboard)) {
    find_object_client_->async_cancel_goal(handle);
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(
        action.action_type, canceled ? CANCELED : TIMEOUT,
        canceled ? "FindObject goal canceled." : "FindObject action result timed out.",
        Json::object(), !canceled, canceled);
  }
  const auto wrapped = result_future.get();
  const auto result = wrapped.result;
  if (!result) {
    return make_skill_result(action.action_type, RECOVERABLE_FAILURE,
                             "FindObject returned no result.", Json::object(), true);
  }
  const bool localized = !result->object_pose.header.frame_id.empty();
  Json payload{{"found", result->found},
               {"approached", result->approached},
               {"localized", localized},
               {"result_code", result->result_code},
               {"object_pose", pose_to_json(result->object_pose)},
               {"observation_pose", pose_to_json(result->observation_pose)},
               {"object_id", result->object_id},
               {"found_room", result->found_room},
               {"found_viewpoint", result->found_viewpoint},
               {"viewpoints_searched", result->viewpoints_searched},
               {"navigation_failures", result->navigation_failures},
               {"selection", parse_json(result->selection_json, Json::object())}};
  using Result = robot_ai_interfaces::action::FindObject::Result;
  std::string outcome = FAILED;
  bool recoverable = false;
  bool fatal = false;
  if (result->result_code == Result::SUCCESS) {
    outcome = SUCCESS;
    blackboard->set<std::string>(bb::TARGET_OBJECT, result->object_id);
    if (localized) {
      blackboard->set<geometry_msgs::msg::PoseStamped>(bb::TARGET_POSE,
                                                       result->object_pose);
      auto memory = bb_get<Json>(blackboard, bb::ENTITY_MEMORY, Json::object());
      memory[result->object_id] = Json{{"type", "object"},
                                       {"pose", pose_to_json(result->object_pose)},
                                       {"room", result->found_room},
                                       {"viewpoint", result->found_viewpoint}};
      blackboard->set<Json>(bb::ENTITY_MEMORY, memory);
    }
  } else if (result->result_code == Result::NOT_FOUND) {
    outcome = NOT_FOUND;
  } else if (result->result_code == Result::TIMEOUT) {
    outcome = TIMEOUT;
    recoverable = true;
  } else if (result->result_code == Result::CANCELED ||
             wrapped.code == rclcpp_action::ResultCode::CANCELED) {
    outcome = CANCELED;
    fatal = true;
  } else if (result->result_code == Result::NAVIGATION_FAILED ||
             result->result_code == Result::PERCEPTION_FAILED) {
    outcome = RECOVERABLE_FAILURE;
    recoverable = true;
  }
  return make_skill_result(action.action_type, outcome, result->message, payload,
                           recoverable, fatal);
}

SkillResultData RemoteSkillClient::find_person_by_attribute(
    const Blackboard::SharedPtr &blackboard, const ActionStepData &action) {
  // Section: Forward the normalized selector and correlation fields unchanged.
  const double timeout =
      action.timeout_sec != 0.0
          ? action.timeout_sec
          : json_value<double>(action.parameters, "timeout_sec", 20.0);
  if (!find_attribute_client_->wait_for_action_server(
          std::chrono::duration<double>(std::min(timeout, 5.0)))) {
    return make_skill_result(
        action.action_type, FAILED,
        "Find-person action unavailable: /gpsr/find_person_by_attribute",
        Json::object(), true);
  }
  robot_ai_interfaces::action::FindPersonByAttribute::Goal goal;
  goal.selector_json = dump_json(action.parameters.value("selector", Json::object()));
  goal.location = !action.target_location.empty()
                      ? action.target_location
                      : json_value<std::string>(action.parameters, "location", "");
  goal.command_id = action.task_id;
  goal.source_step_id = action.step_id;
  goal.timeout_sec = static_cast<float>(timeout);
  goal.confirmation_min_frames = static_cast<std::uint32_t>(
      json_value<int>(action.parameters, "confirmation_min_frames", 3));
  auto goal_future = find_attribute_client_->async_send_goal(goal);
  if (!wait_future(goal_future, std::min(timeout, 5.0), blackboard)) {
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(
        action.action_type, canceled ? CANCELED : TIMEOUT,
        "Find-person goal acceptance did not complete before the absolute deadline.",
        Json::object(), !canceled, canceled);
  }
  auto handle = goal_future.get();
  if (!handle) {
    return make_skill_result(action.action_type, FAILED,
                             "Find-person goal was rejected.", Json::object(), true);
  }
  auto result_future = find_attribute_client_->async_get_result(handle);
  if (!wait_future(result_future, timeout, blackboard)) {
    find_attribute_client_->async_cancel_goal(handle);
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(
        action.action_type, canceled ? CANCELED : TIMEOUT,
        "Find-person action did not finish before its absolute deadline.",
        Json::object(), !canceled, canceled);
  }
  const auto result = result_future.get().result;
  if (!result)
    return make_skill_result(action.action_type, FAILED,
                             "Find-person returned no result.", Json::object(), true);
  const auto person_ref = parse_json(result->person_ref_json, Json::object());
  const auto observation = parse_json(result->observation_json, Json::object());
  if (person_ref.is_object() && !person_ref.empty()) {
    blackboard->set<Json>(bb::PERSON_REF, person_ref);
    blackboard->set<std::string>(
        bb::TARGET_PERSON,
        person_ref.value("profile_id", person_ref.value("track_id", "")));
  }
  if (!result->memory_id.empty()) {
    blackboard->set<std::string>(bb::GPSR_GUEST_MEMORY_ID, result->memory_id);
    blackboard->set<std::uint64_t>(bb::GPSR_GUEST_MEMORY_REVISION,
                                   result->memory_revision);
  }
  const std::set<std::string> valid{SUCCESS,   FAILED,  INVALID_INPUT,
                                    NOT_FOUND, TIMEOUT, CANCELED};
  const auto outcome = valid.count(result->outcome) ? result->outcome : FAILED;
  return make_skill_result(
      action.action_type, outcome, result->message,
      Json{{"person_ref", person_ref},
           {"observation", observation},
           {"memory_id", result->memory_id},
           {"memory_revision", result->memory_revision}},
      outcome == FAILED || outcome == NOT_FOUND || outcome == TIMEOUT,
      outcome == CANCELED || outcome == INVALID_INPUT);
}

SkillResultData RemoteSkillClient::extract_guest_info(
    const Blackboard::SharedPtr &blackboard, const ActionStepData &action) {
  // Section: Preserve field/reference selection and result mirroring.
  const double timeout = action.timeout_sec != 0.0 ? action.timeout_sec : 10.0;
  if (!extract_client_->wait_for_action_server(
          std::chrono::duration<double>(std::min(timeout, 5.0)))) {
    return make_skill_result(
        action.action_type, FAILED,
        "Extract-guest action unavailable: /gpsr/extract_guest_info", Json::object(),
        true);
  }
  Json fields = action.parameters.value("fields", Json::array());
  if ((!fields.is_array() || fields.empty()) && action.parameters.contains("field")) {
    fields = Json::array({action.parameters.at("field")});
  }
  Json person_ref = action.parameters.value(
      "person_ref", bb_get<Json>(blackboard, bb::PERSON_REF, Json::object()));
  std::string memory_id = json_value<std::string>(
      action.parameters, "memory_id",
      bb_get<std::string>(blackboard, bb::GPSR_GUEST_MEMORY_ID, ""));
  robot_ai_interfaces::action::ExtractGuestInfo::Goal goal;
  goal.memory_id = memory_id;
  goal.person_ref_json = person_ref.is_object() ? dump_json(person_ref) : "{}";
  goal.command_id = action.task_id;
  goal.source_step_id = action.step_id;
  if (fields.is_array()) {
    for (const auto &field : fields) goal.fields.push_back(field.get<std::string>());
  }
  goal.timeout_sec = static_cast<float>(timeout);
  auto goal_future = extract_client_->async_send_goal(goal);
  if (!wait_future(goal_future, timeout, blackboard) || !goal_future.get()) {
    return make_skill_result(action.action_type, FAILED,
                             "Extract-guest goal was rejected or timed out.",
                             Json::object(), true);
  }
  auto handle = goal_future.get();
  auto result_future = extract_client_->async_get_result(handle);
  if (!wait_future(result_future, timeout, blackboard)) {
    extract_client_->async_cancel_goal(handle);
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(action.action_type, canceled ? CANCELED : TIMEOUT,
                             "Extract-guest action timed out.", Json::object(),
                             !canceled, canceled);
  }
  const auto result = result_future.get().result;
  if (!result)
    return make_skill_result(action.action_type, FAILED,
                             "Extract-guest returned no result.", Json::object(), true);
  const auto info = parse_json(result->info_json, Json::object());
  blackboard->set<Json>(bb::EXTRACTED_PERSON_INFO, info);
  if (!result->memory_id.empty()) {
    blackboard->set<std::string>(bb::GPSR_GUEST_MEMORY_ID, result->memory_id);
    blackboard->set<std::uint64_t>(bb::GPSR_GUEST_MEMORY_REVISION,
                                   result->memory_revision);
  }
  const std::set<std::string> valid{SUCCESS,   FAILED,  INVALID_INPUT,
                                    NOT_FOUND, TIMEOUT, CANCELED};
  const auto outcome = valid.count(result->outcome) ? result->outcome : FAILED;
  Json payload = info.is_object() ? info : Json::object();
  payload["guest_info"] = info;
  payload["memory_id"] = result->memory_id;
  payload["memory_revision"] = result->memory_revision;
  payload["missing_fields"] = result->missing_fields;
  return make_skill_result(
      action.action_type, outcome, result->message, payload,
      outcome == FAILED || outcome == NOT_FOUND || outcome == TIMEOUT,
      outcome == CANCELED || outcome == INVALID_INPUT);
}

ManipulationClient::ManipulationClient(const rclcpp::Node::SharedPtr &node,
                                       bool simulate, std::string pick_action,
                                       std::string place_action,
                                       std::string handover_action)
    : node_(node), simulate_(simulate) {
  // Section: Simulation deliberately owns no ROS action handles.
  if (node_ && !simulate_) {
    pick_client_ =
        rclcpp_action::create_client<robot_ai_interfaces::action::PickObject>(
            node_, std::move(pick_action));
    place_client_ =
        rclcpp_action::create_client<robot_ai_interfaces::action::PlaceObject>(
            node_, std::move(place_action));
    handover_client_ =
        rclcpp_action::create_client<robot_ai_interfaces::action::Handover>(
            node_, std::move(handover_action));
  }
}

SkillResultData ManipulationClient::pick_object(const Blackboard::SharedPtr &blackboard,
                                                const ActionStepData &action) {
  // Section: Resolve the object and grasp pose already validated by the skill state.
  const auto object = !action.target_object.empty()
                          ? action.target_object
                          : bb_get<std::string>(blackboard, bb::TARGET_OBJECT, "");
  if (simulate_) {
    if (json_value<bool>(action.parameters, "simulate_failure", false)) {
      return make_skill_result(action.action_type, FAILED, "Pick failed.",
                               Json::object(), true);
    }
    blackboard->set<std::string>(bb::HELD_OBJECT, object);
    return make_skill_result(action.action_type, SUCCESS, "Object picked.",
                             Json{{"held_object", object}, {"simulated", true}});
  }
  const double timeout = action.timeout_sec > 0.0 ? action.timeout_sec : 20.0;
  if (!pick_client_ || !pick_client_->wait_for_action_server(
                           std::chrono::duration<double>(std::min(timeout, 5.0)))) {
    return make_skill_result(action.action_type, FAILED,
                             "PickObject action unavailable.", Json::object(), true);
  }
  auto pose = bb_get<geometry_msgs::msg::PoseStamped>(
      blackboard, bb::TARGET_POSE, geometry_msgs::msg::PoseStamped{});
  if (pose.header.frame_id.empty() && action.parameters.contains("object_pose")) {
    const auto parsed = pose_from_json(action.parameters.at("object_pose"));
    if (parsed) pose = *parsed;
  }
  robot_ai_interfaces::action::PickObject::Goal goal;
  goal.object_id = object;
  goal.object_pose = pose;
  goal.timeout_sec = static_cast<float>(timeout);
  auto goal_future = pick_client_->async_send_goal(goal);
  if (!wait_future(goal_future, std::min(timeout, 5.0), blackboard) ||
      !goal_future.get()) {
    return make_skill_result(action.action_type, FAILED, "Pick failed.", Json::object(),
                             true);
  }
  auto handle = goal_future.get();
  auto result_future = pick_client_->async_get_result(handle);
  if (!wait_future(result_future, timeout, blackboard)) {
    pick_client_->async_cancel_goal(handle);
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(action.action_type, canceled ? CANCELED : TIMEOUT,
                             canceled ? "Pick canceled." : "Pick timed out.",
                             Json::object(), !canceled, canceled);
  }
  const auto wrapped = result_future.get();
  const auto result = wrapped.result;
  if (!result || !result->success) {
    return make_skill_result(action.action_type, FAILED,
                             result ? result->message : "Pick failed.", Json::object(),
                             true);
  }
  const auto held = result->held_object_id.empty() ? object : result->held_object_id;
  blackboard->set<std::string>(bb::HELD_OBJECT, held);
  return make_skill_result(action.action_type, SUCCESS, result->message,
                           Json{{"held_object", held}});
}

SkillResultData ManipulationClient::place_object(
    const Blackboard::SharedPtr &blackboard, const ActionStepData &action) {
  // Section: Clear held state only after the placement action confirms success.
  const auto held = bb_get<std::string>(blackboard, bb::HELD_OBJECT, "");
  if (simulate_) {
    blackboard->set<std::string>(bb::HELD_OBJECT, "");
    return make_skill_result(action.action_type, SUCCESS, "Object placed.",
                             Json{{"object", held}, {"simulated", true}});
  }
  const double timeout = action.timeout_sec > 0.0 ? action.timeout_sec : 20.0;
  if (!place_client_ || !place_client_->wait_for_action_server(
                            std::chrono::duration<double>(std::min(timeout, 5.0)))) {
    return make_skill_result(action.action_type, FAILED,
                             "PlaceObject action unavailable.", Json::object(), true);
  }
  auto pose = bb_get<geometry_msgs::msg::PoseStamped>(
      blackboard, bb::TARGET_POSE, geometry_msgs::msg::PoseStamped{});
  if (pose.header.frame_id.empty() && action.parameters.contains("target_pose")) {
    const auto parsed = pose_from_json(action.parameters.at("target_pose"));
    if (parsed) pose = *parsed;
  }
  robot_ai_interfaces::action::PlaceObject::Goal goal;
  goal.object_id = held;
  goal.target_pose = pose;
  goal.timeout_sec = static_cast<float>(timeout);
  auto goal_future = place_client_->async_send_goal(goal);
  if (!wait_future(goal_future, std::min(timeout, 5.0), blackboard) ||
      !goal_future.get()) {
    return make_skill_result(action.action_type, FAILED,
                             "PlaceObject goal was rejected or timed out.",
                             Json::object(), true);
  }
  auto handle = goal_future.get();
  auto result_future = place_client_->async_get_result(handle);
  if (!wait_future(result_future, timeout, blackboard)) {
    place_client_->async_cancel_goal(handle);
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(action.action_type, canceled ? CANCELED : TIMEOUT,
                             canceled ? "Place canceled." : "Place timed out.",
                             Json::object(), !canceled, canceled);
  }
  const auto result = result_future.get().result;
  if (!result || !result->success) {
    return make_skill_result(action.action_type, FAILED,
                             result ? result->message : "Place failed.", Json::object(),
                             true);
  }
  blackboard->set<std::string>(bb::HELD_OBJECT, "");
  return make_skill_result(action.action_type, SUCCESS, result->message,
                           Json{{"object", held}});
}

SkillResultData ManipulationClient::dispose_trash(
    const Blackboard::SharedPtr &blackboard, const ActionStepData &action) {
  // Section: Preserve held-or-target selection and clearing behavior.
  auto object = bb_get<std::string>(blackboard, bb::HELD_OBJECT, "");
  if (object.empty()) {
    object = action.target_object;
  }
  blackboard->set<std::string>(bb::HELD_OBJECT, "");
  return make_skill_result(action.action_type, SUCCESS, "Trash disposed.",
                           Json{{"object", object}});
}

SkillResultData ManipulationClient::handover(const Blackboard::SharedPtr &blackboard,
                                             const ActionStepData &action) {
  // Section: Resolve the validated recipient before sending the handover goal.
  const auto held = bb_get<std::string>(blackboard, bb::HELD_OBJECT, "");
  const auto recipient = !action.target_person.empty()
                             ? action.target_person
                             : bb_get<std::string>(blackboard, bb::TARGET_PERSON, "");
  if (simulate_) {
    blackboard->set<std::string>(bb::HELD_OBJECT, "");
    return make_skill_result(
        action.action_type, SUCCESS, "Object handed over.",
        Json{{"object", held}, {"recipient", recipient}, {"simulated", true}});
  }
  const double timeout = action.timeout_sec > 0.0 ? action.timeout_sec : 20.0;
  if (!handover_client_ || !handover_client_->wait_for_action_server(
                               std::chrono::duration<double>(std::min(timeout, 5.0)))) {
    return make_skill_result(action.action_type, FAILED, "Handover action unavailable.",
                             Json::object(), true);
  }
  robot_ai_interfaces::action::Handover::Goal goal;
  goal.object_id = held;
  goal.recipient = recipient;
  goal.timeout_sec = static_cast<float>(timeout);
  auto goal_future = handover_client_->async_send_goal(goal);
  if (!wait_future(goal_future, std::min(timeout, 5.0), blackboard) ||
      !goal_future.get()) {
    return make_skill_result(action.action_type, FAILED,
                             "Handover goal was rejected or timed out.", Json::object(),
                             true);
  }
  auto handle = goal_future.get();
  auto result_future = handover_client_->async_get_result(handle);
  if (!wait_future(result_future, timeout, blackboard)) {
    handover_client_->async_cancel_goal(handle);
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(action.action_type, canceled ? CANCELED : TIMEOUT,
                             canceled ? "Handover canceled." : "Handover timed out.",
                             Json::object(), !canceled, canceled);
  }
  const auto result = result_future.get().result;
  if (!result || !result->success) {
    return make_skill_result(action.action_type, FAILED,
                             result ? result->message : "Handover failed.",
                             Json::object(), true);
  }
  blackboard->set<std::string>(bb::HELD_OBJECT, "");
  return make_skill_result(action.action_type, SUCCESS, result->message,
                           Json{{"object", held}, {"recipient", recipient}});
}

GuidePersonClient::GuidePersonClient(const rclcpp::Node::SharedPtr &node, bool simulate,
                                     std::string action_name)
    : node_(node), simulate_(simulate), action_name_(std::move(action_name)) {
  // Section: Simulation deliberately owns no ROS action handle.
  if (node_ && !simulate_) {
    client_ = rclcpp_action::create_client<robot_ai_interfaces::action::GuidePerson>(
        node_, action_name_);
  }
}

SkillResultData GuidePersonClient::guide(const Blackboard::SharedPtr &blackboard,
                                         const ActionStepData &action) {
  // Section: Normalize the two accepted destination representations.
  const auto destination =
      !action.target_location.empty()
          ? action.target_location
          : json_value<std::string>(action.parameters, "destination", "");
  if (simulate_) {
    return make_skill_result(action.action_type, SUCCESS, "Guide completed.",
                             Json{{"person", action.target_person},
                                  {"destination", destination},
                                  {"simulated", true}});
  }
  const double timeout = action.timeout_sec > 0.0 ? action.timeout_sec : 60.0;
  if (!client_ || !client_->wait_for_action_server(
                      std::chrono::duration<double>(std::min(timeout, 5.0)))) {
    return make_skill_result(action.action_type, FAILED,
                             "GuidePerson action unavailable: " + action_name_,
                             Json::object(), true);
  }
  robot_ai_interfaces::action::GuidePerson::Goal goal;
  goal.target_person = action.target_person;
  goal.destination = destination;
  goal.timeout_sec = static_cast<float>(timeout);
  auto goal_future = client_->async_send_goal(goal);
  if (!wait_future(goal_future, std::min(timeout, 5.0), blackboard) ||
      !goal_future.get()) {
    return make_skill_result(action.action_type, FAILED,
                             "GuidePerson goal was rejected or timed out.",
                             Json::object(), true);
  }
  auto handle = goal_future.get();
  auto result_future = client_->async_get_result(handle);
  if (!wait_future(result_future, timeout, blackboard)) {
    client_->async_cancel_goal(handle);
    const bool canceled = cancel_requested(blackboard);
    return make_skill_result(action.action_type, canceled ? CANCELED : TIMEOUT,
                             canceled ? "Guide canceled." : "Guide timed out.",
                             Json::object(), !canceled, canceled);
  }
  const auto wrapped = result_future.get();
  const auto result = wrapped.result;
  if (!result || !result->success) {
    const auto outcome = result && result->person_lost ? PERSON_LOST : FAILED;
    return make_skill_result(action.action_type, outcome,
                             result ? result->message : "Guide failed.", Json::object(),
                             true);
  }
  return make_skill_result(
      action.action_type, SUCCESS, result->message,
      Json{{"person", action.target_person}, {"destination", destination}});
}

LifecycleResourceClient::LifecycleResourceClient(const rclcpp::Node::SharedPtr &node,
                                                 double transition_timeout_sec,
                                                 double operation_margin_sec)
    : node_(node),
      transition_timeout_sec_(transition_timeout_sec),
      operation_margin_sec_(operation_margin_sec) {
  // Section: Bind only the four established lifecycle-manager endpoints.
  prepare_client_ =
      node_->create_client<robot_ai_interfaces::srv::ManageLifecycleSkill>(
          GPSR_PREPARE_SKILL_SERVICE);
  cleanup_client_ =
      node_->create_client<robot_ai_interfaces::srv::ManageLifecycleSkill>(
          GPSR_CLEANUP_SKILL_SERVICE);
  recover_client_ =
      node_->create_client<robot_ai_interfaces::srv::ManageLifecycleSkill>(
          GPSR_RECOVER_SKILL_SERVICE);
}

std::pair<bool, std::string> LifecycleResourceClient::call(
    const rclcpp::Client<robot_ai_interfaces::srv::ManageLifecycleSkill>::SharedPtr
        &client,
    const std::string &skill_name) {
  // Section: Use one manager deadline plus the original response margin.
  if (!client->wait_for_service(
          std::chrono::duration<double>(std::min(5.0, transition_timeout_sec_)))) {
    return {false, "GPSR lifecycle manager service is unavailable"};
  }
  auto request =
      std::make_shared<robot_ai_interfaces::srv::ManageLifecycleSkill::Request>();
  request->skill_name = skill_name;
  request->timeout_sec = static_cast<float>(transition_timeout_sec_);
  auto future = client->async_send_request(request);
  if (!wait_future(future, transition_timeout_sec_ + operation_margin_sec_)) {
    return {false, "GPSR lifecycle manager response timed out"};
  }
  const auto response = future.get();
  return {response && response->success,
          response ? response->message : "GPSR lifecycle manager failed"};
}

std::pair<bool, std::string> LifecycleResourceClient::start(
    const std::string &skill_name) {
  // Section: Forward prepare without local lifecycle-node ownership.
  return call(prepare_client_, skill_name);
}

std::pair<bool, std::string> LifecycleResourceClient::stop(
    const std::string &skill_name) {
  // Section: Forward cleanup as the queue's result-commit barrier.
  return call(cleanup_client_, skill_name);
}

std::pair<bool, std::string> LifecycleResourceClient::recover(
    const std::string &skill_name) {
  // Section: Forward bounded manager recovery without inventing retry policy.
  return call(recover_client_, skill_name);
}

}  // namespace robot_yasmin_tasks
