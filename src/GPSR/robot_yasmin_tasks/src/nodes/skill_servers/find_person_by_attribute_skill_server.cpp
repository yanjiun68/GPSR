#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <future>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <receptionist_interfaces/msg/tracked_person_appearance_array.hpp>
#include <robot_ai_interfaces/action/find_person_by_attribute.hpp>
#include <robot_ai_interfaces/srv/save_gpsr_guest.hpp>

namespace robot_yasmin_tasks {

using FindAction = robot_ai_interfaces::action::FindPersonByAttribute;
using FindGoalHandle = rclcpp_action::ServerGoalHandle<FindAction>;
using Observation = receptionist_interfaces::msg::TrackedPersonAppearance;
using ObservationArray = receptionist_interfaces::msg::TrackedPersonAppearanceArray;
using Json = nlohmann::json;

// Special function: Normalize spoken selector values for deterministic comparisons.
std::string normalize(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    if (character == ' ' || character == '-') return '_';
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

// Special function: Convert tri-state appearance evidence to its JSON contract.
std::string appearance_state(const std::uint8_t state) {
  return state == 2U ? "yes" : state == 1U ? "no" : "unknown";
}

// Special function: Serialize a tracked observation without retaining ROS-owned memory.
Json observation_json(const Observation &person) {
  return Json{{"tracking_session_id", person.tracking_session_id},
              {"track_id", person.track_id},
              {"person_id", person.person_id},
              {"profile_id", person.profile_id},
              {"person_uuid", person.person_uuid},
              {"bbox_xyxy", person.bbox_xyxy},
              {"appearance", {{"glasses", appearance_state(person.appearance.glasses)},
                               {"glasses_confidence", person.appearance.glasses_confidence},
                               {"hat", appearance_state(person.appearance.hat)},
                               {"hat_confidence", person.appearance.hat_confidence},
                               {"face_mask", appearance_state(person.appearance.face_mask)},
                               {"face_mask_confidence", person.appearance.face_mask_confidence},
                               {"shirt_color", person.appearance.shirt_color.empty() ? "unknown" : person.appearance.shirt_color},
                               {"shirt_color_confidence", person.appearance.shirt_color_confidence}}},
              {"pose", person.pose.empty() ? "unknown" : person.pose},
              {"pose_confidence", person.pose_confidence},
              {"gesture", person.gesture.empty() ? "none" : person.gesture},
              {"gesture_confidence", person.gesture_confidence},
              {"stamp", {{"sec", person.header.stamp.sec}, {"nanosec", person.header.stamp.nanosec}}}};
}

// Special function: Score one positive predicate and reject mismatches with a negative value.
double predicate_score(const Observation &person, const Json &predicate) {
  const auto type = normalize(predicate.value("type", std::string()));
  auto value = normalize(predicate.value("value", std::string()));
  if (type == "any_person") return 1.0;
  if (type == "pose") {
    if (value == "lying") value = "lying_down";
    return normalize(person.pose) == value ? person.pose_confidence : -1.0;
  }
  if (type == "gesture") {
    const auto actual = normalize(person.gesture);
    const std::map<std::string, std::set<std::string>> aliases{
        {"right_hand_raised", {"right_hand_raised", "raising_right_arm"}},
        {"left_hand_raised", {"left_hand_raised", "raising_left_arm"}},
        {"waving", {"waving", "left_arm_waving", "right_arm_waving", "both_arms_waving"}}};
    const auto iterator = aliases.find(value);
    const bool matches = iterator == aliases.end() ? actual == value : iterator->second.count(actual) != 0U;
    return matches ? person.gesture_confidence : -1.0;
  }
  if (type == "shirt_color") {
    return normalize(person.appearance.shirt_color) == value
               ? person.appearance.shirt_color_confidence : -1.0;
  }
  if (type == "glasses") return person.appearance.glasses == 2U ? person.appearance.glasses_confidence : -1.0;
  if (type == "hat") return person.appearance.hat == 2U ? person.appearance.hat_confidence : -1.0;
  if (type == "face_mask") return person.appearance.face_mask == 2U ? person.appearance.face_mask_confidence : -1.0;
  return -1.0;
}

class FindPersonByAttributeServer final : public rclcpp_lifecycle::LifecycleNode {
 public:
  // Special function: Declare lifecycle configuration without acquiring perception resources.
  FindPersonByAttributeServer() : LifecycleNode("find_person_by_attribute_skill_server") {
    declare_parameter<std::string>("action_name", "/gpsr/find_person_by_attribute");
    declare_parameter<std::string>("appearance_topic", "/appearance/tracked_people");
    declare_parameter<std::string>("memory_save_service", "/gpsr/guest_memory/save");
    declare_parameter<double>("default_timeout_sec", 20.0);
    declare_parameter<std::int64_t>("confirmation_min_frames", 3);
    declare_parameter<double>("memory_timeout_sec", 2.0);
    declare_parameter<std::vector<std::string>>("enabled_wearable_predicates", {"glasses"});
  }

  // Special function: Allocate the action, memory, and appearance interfaces together.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    default_timeout_sec_ = get_parameter("default_timeout_sec").as_double();
    confirmation_min_frames_ = std::max<std::int64_t>(1, get_parameter("confirmation_min_frames").as_int());
    memory_timeout_sec_ = std::max(0.1, get_parameter("memory_timeout_sec").as_double());
    enabled_wearables_.clear();
    for (const auto &item : get_parameter("enabled_wearable_predicates").as_string_array()) {
      enabled_wearables_.insert(normalize(item));
    }
    subscription_ = create_subscription<ObservationArray>(
        get_parameter("appearance_topic").as_string(), 10,
        std::bind(&FindPersonByAttributeServer::observe, this, std::placeholders::_1));
    memory_client_ = create_client<robot_ai_interfaces::srv::SaveGpsrGuest>(
        get_parameter("memory_save_service").as_string());
    if (!memory_client_->wait_for_service(std::chrono::duration<double>(memory_timeout_sec_))) {
      release_resources();
      return CallbackReturn::FAILURE;
    }
    action_server_ = rclcpp_action::create_server<FindAction>(
        this, get_parameter("action_name").as_string(),
        std::bind(&FindPersonByAttributeServer::admit, this, std::placeholders::_1, std::placeholders::_2),
        [](const std::shared_ptr<FindGoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        std::bind(&FindPersonByAttributeServer::execute_async, this, std::placeholders::_1));
    return CallbackReturn::SUCCESS;
  }

  // Special function: Open goal admission only in the ACTIVE lifecycle state.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    active_.store(true);
    return CallbackReturn::SUCCESS;
  }

  // Special function: Close admission and wake any bounded observation wait.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    active_.store(false);
    condition_.notify_all();
    return busy_.load() ? CallbackReturn::FAILURE : CallbackReturn::SUCCESS;
  }

  // Special function: Release configured resources only after the current goal exits.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    if (busy_.load()) return CallbackReturn::FAILURE;
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  // Special function: Fail closed and release every configured resource.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    active_.store(false);
    condition_.notify_all();
    release_resources();
    return CallbackReturn::SUCCESS;
  }

 private:
  // Special function: Append one frame to a bounded sequence and wake the mission worker.
  void observe(const ObservationArray::SharedPtr message) {
    if (!active_.load()) return;
    std::lock_guard<std::mutex> lock(observation_mutex_);
    ++sequence_;
    frames_.push_back({sequence_, *message});
    if (frames_.size() > 120U) frames_.erase(frames_.begin());
    condition_.notify_all();
  }

  // Special function: Validate selector syntax before reserving the single goal slot.
  rclcpp_action::GoalResponse admit(const rclcpp_action::GoalUUID &,
                                    const std::shared_ptr<const FindAction::Goal> goal) {
    if (!active_.load() || busy_.exchange(true)) return rclcpp_action::GoalResponse::REJECT;
    try {
      const auto payload = Json::parse(goal->selector_json.empty() ? "{}" : goal->selector_json);
      const auto predicates = payload.contains("all") ? payload.at("all") : Json::array({payload});
      if (!predicates.is_array() || predicates.empty()) throw std::runtime_error("empty selector");
      for (const auto &predicate : predicates) {
        const auto type = normalize(predicate.value("type", std::string()));
        const auto operation = normalize(predicate.value("operator", std::string("equals")));
        const auto value = normalize(predicate.value("value", std::string()));
        const std::set<std::string> supported{"any_person", "pose", "gesture", "shirt_color", "glasses", "hat", "face_mask"};
        if (supported.count(type) == 0U || (operation != "equals" && operation != "is")) throw std::runtime_error("unsupported selector");
        if ((type == "glasses" || type == "hat" || type == "face_mask") &&
            (enabled_wearables_.count(type) == 0U || (value != "yes" && value != "true" && value != "wearing" && value != "1"))) {
          throw std::runtime_error("unsupported wearable selector");
        }
      }
    } catch (const std::exception &) {
      busy_.store(false);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Special function: Move blocking confirmation and memory persistence off executor callbacks.
  void execute_async(const std::shared_ptr<FindGoalHandle> goal_handle) {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  // Special function: Confirm one canonical track across consecutive matching frames.
  void execute(const std::shared_ptr<FindGoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();
    const auto payload = Json::parse(goal->selector_json.empty() ? "{}" : goal->selector_json);
    const auto predicates = payload.contains("all") ? payload.at("all") : Json::array({payload});
    const auto required = std::max<std::uint32_t>(1, goal->confirmation_min_frames == 0U
                                                        ? static_cast<std::uint32_t>(confirmation_min_frames_)
                                                        : goal->confirmation_min_frames);
    const auto timeout = goal->timeout_sec > 0.0F ? goal->timeout_sec : default_timeout_sec_;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(std::max(0.1, timeout));
    std::map<std::string, std::uint32_t> confirmations;
    std::uint64_t cursor = 0;
    bool saw_frames = false;
    bool saw_people = false;
    std::optional<Observation> match;
    while (std::chrono::steady_clock::now() < deadline && active_.load() && !goal_handle->is_canceling()) {
      ObservationArray frame;
      {
        std::unique_lock<std::mutex> lock(observation_mutex_);
        condition_.wait_for(lock, std::chrono::milliseconds(100), [this, cursor]() { return sequence_ > cursor || !active_.load(); });
        const auto iterator = std::find_if(frames_.begin(), frames_.end(), [cursor](const auto &item) { return item.first > cursor; });
        if (iterator == frames_.end()) continue;
        cursor = iterator->first;
        frame = iterator->second;
      }
      saw_frames = true;
      saw_people = saw_people || !frame.people.empty();
      std::set<std::string> matched_keys;
      for (const auto &person : frame.people) {
        bool matches = true;
        for (const auto &predicate : predicates) matches = matches && predicate_score(person, predicate) >= 0.0;
        if (!matches) continue;
        const auto key = person.tracking_session_id + ":" + std::to_string(person.track_id);
        matched_keys.insert(key);
        if (++confirmations[key] >= required) {
          match = person;
          break;
        }
      }
      for (auto &[key, count] : confirmations) if (matched_keys.count(key) == 0U) count = 0;
      auto feedback = std::make_shared<FindAction::Feedback>();
      feedback->status = match ? "confirming" : "searching";
      feedback->visible_count = static_cast<std::uint32_t>(frame.people.size());
      feedback->matching_count = static_cast<std::uint32_t>(matched_keys.size());
      goal_handle->publish_feedback(feedback);
      if (match) break;
    }
    auto result = std::make_shared<FindAction::Result>();
    if (goal_handle->is_canceling() || !active_.load()) {
      result->outcome = "canceled";
      result->message = "Find-person goal canceled.";
      goal_handle->canceled(result);
    } else if (!match) {
      result->outcome = saw_frames ? "not_found" : "failed";
      result->message = saw_people ? "People were visible, but none matched every requested attribute."
                                   : saw_frames ? "Fresh tracked-person frames arrived, but no people were visible."
                                                : "No fresh tracked-person observations arrived before the deadline.";
      goal_handle->abort(result);
    } else {
      persist_match(goal_handle, *match, predicates, result);
    }
    busy_.store(false);
  }

  // Special function: Make guest persistence a mandatory part of find success.
  void persist_match(const std::shared_ptr<FindGoalHandle> &goal_handle, const Observation &person,
                     const Json &predicates, const std::shared_ptr<FindAction::Result> &result) {
    auto snapshot = observation_json(person);
    auto person_ref = snapshot;
    person_ref["location"] = goal_handle->get_goal()->location;
    person_ref["selector"] = Json{{"all", predicates}};
    auto request = std::make_shared<robot_ai_interfaces::srv::SaveGpsrGuest::Request>();
    request->record.command_id = goal_handle->get_goal()->command_id;
    request->record.source_step_id = goal_handle->get_goal()->source_step_id;
    request->record.tracking_session_id = person.tracking_session_id;
    request->record.track_id = person.track_id;
    request->record.person_ref_json = person_ref.dump();
    request->record.guest_info_json = Json{{"pose", snapshot["pose"]}, {"gesture", snapshot["gesture"]},
                                           {"profile_id", snapshot["profile_id"]}, {"person_id", snapshot["person_id"]},
                                           {"track_id", snapshot["track_id"]}, {"appearance", snapshot["appearance"]}}.dump();
    request->record.profile = person.profile;
    if (!memory_client_->service_is_ready()) {
      result->outcome = "failed";
      result->message = "GPSR guest-memory save service is unavailable.";
      goal_handle->abort(result);
      return;
    }
    auto future = memory_client_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(memory_timeout_sec_)) != std::future_status::ready) {
      result->outcome = "failed";
      result->message = "GPSR guest-memory save timed out.";
      goal_handle->abort(result);
      return;
    }
    const auto response = future.get();
    if (!response->success) {
      result->outcome = "failed";
      result->message = response->message;
      goal_handle->abort(result);
      return;
    }
    result->success = true;
    result->outcome = "success";
    result->person_ref_json = person_ref.dump();
    result->memory_id = response->record.memory_id;
    result->memory_revision = response->record.revision;
    result->observation_json = snapshot.dump();
    result->message = "Person matching the requested attributes was found and stored.";
    goal_handle->succeed(result);
  }

  // Special function: Drop all configured handles and buffered observations.
  void release_resources() {
    action_server_.reset();
    subscription_.reset();
    memory_client_.reset();
    std::lock_guard<std::mutex> lock(observation_mutex_);
    frames_.clear();
    sequence_ = 0;
    busy_.store(false);
  }

  std::atomic_bool active_{false};
  std::atomic_bool busy_{false};
  double default_timeout_sec_{20.0};
  std::int64_t confirmation_min_frames_{3};
  double memory_timeout_sec_{2.0};
  std::set<std::string> enabled_wearables_{"glasses"};
  std::mutex observation_mutex_;
  std::condition_variable condition_;
  std::uint64_t sequence_{0};
  std::vector<std::pair<std::uint64_t, ObservationArray>> frames_;
  rclcpp::Subscription<ObservationArray>::SharedPtr subscription_;
  rclcpp::Client<robot_ai_interfaces::srv::SaveGpsrGuest>::SharedPtr memory_client_;
  rclcpp_action::Server<FindAction>::SharedPtr action_server_;
};

}  // namespace robot_yasmin_tasks

int main(int argc, char **argv) {
  // Section: Concurrent callbacks keep perception and memory futures live during action work.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_yasmin_tasks::FindPersonByAttributeServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
