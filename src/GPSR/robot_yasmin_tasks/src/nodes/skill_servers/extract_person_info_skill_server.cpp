// Section: Lifecycle action server that reads GPSR guest memory and returns requested
// person fields.
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_ai_interfaces/action/extract_guest_info.hpp>
#include <robot_ai_interfaces/msg/tracked_person_appearance_array.hpp>
#include <robot_ai_interfaces/srv/get_gpsr_guest.hpp>
#include <robot_ai_interfaces/srv/get_profile.hpp>
#include <robot_ai_interfaces/srv/save_gpsr_guest.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace robot_yasmin_tasks {

using Action = robot_ai_interfaces::action::ExtractGuestInfo;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
using GuestRecord = robot_ai_interfaces::msg::GpsrGuestRecord;
using Json = nlohmann::json;

// Special function: Parse only a JSON object at an action or memory trust boundary.
Json parse_json_object(const std::string &raw) {
  try {
    auto value = Json::parse(raw.empty() ? "{}" : raw);
    return value.is_object() ? value : Json::object();
  } catch (const Json::exception &) {
    return Json::object();
  }
}

// Special function: Normalize extraction field aliases used by natural-language plans.
// Section: Field normalization and lifecycle request handling for guest-memory
// extraction.
std::string normalize_field(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char character) {
                   return static_cast<char>(std::tolower(character));
                 });
  if (value == "favourite_drink") return "favorite_drink";
  if (value == "shirt_colour") return "shirt_color";
  return value;
}

// Special function: Copy known registered fields into a JSON information map.
void merge_profile(Json &info, const robot_ai_interfaces::msg::PersonProfile &profile) {
  auto add_text = [&info](const char *name, const std::string &value) {
    if (!value.empty() && value != "unknown") info[name] = value;
  };
  add_text("profile_id", profile.profile_id);
  add_text("name", profile.name);
  add_text("gender", profile.gender);
  add_text("favorite_drink", profile.favorite_drink);
  add_text("hometown", profile.hometown);
  if (profile.age > 0) info["age"] = profile.age;
  const auto state_text = [](std::uint8_t state) {
    return state == 2U ? "yes" : state == 1U ? "no" : "unknown";
  };
  if (profile.appearance.glasses != 0U)
    info["glasses"] = state_text(profile.appearance.glasses);
  if (profile.appearance.hat != 0U) info["hat"] = state_text(profile.appearance.hat);
  if (profile.appearance.face_mask != 0U)
    info["face_mask"] = state_text(profile.appearance.face_mask);
  add_text("shirt_color", profile.appearance.shirt_color);
}

class ExtractPersonInfoServer final : public rclcpp_lifecycle::LifecycleNode {
 public:
  // Special function: Declare stable endpoints while leaving the server UNCONFIGURED.
  ExtractPersonInfoServer() : LifecycleNode("extract_person_info_skill_server") {
    declare_parameter<std::string>("action_name", "/gpsr/extract_guest_info");
    declare_parameter<std::string>("appearance_topic", "/appearance/tracked_people");
    declare_parameter<std::string>("memory_get_service", "/gpsr/guest_memory/get");
    declare_parameter<std::string>("memory_save_service", "/gpsr/guest_memory/save");
    declare_parameter<std::string>("profile_get_service", "/gpsr/identity/get_profile");
    declare_parameter<double>("memory_timeout_sec", 2.0);
    declare_parameter<double>("live_stale_timeout_sec", 1.0);
  }

  // Special function: Allocate optional live perception and mandatory memory
  // interfaces.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    memory_timeout_sec_ =
        std::max(0.1, get_parameter("memory_timeout_sec").as_double());
    live_stale_timeout_sec_ =
        std::max(0.1, get_parameter("live_stale_timeout_sec").as_double());
    subscription_ =
        create_subscription<robot_ai_interfaces::msg::TrackedPersonAppearanceArray>(
            get_parameter("appearance_topic").as_string(), 10,
            std::bind(&ExtractPersonInfoServer::observe, this, std::placeholders::_1));
    get_client_ = create_client<robot_ai_interfaces::srv::GetGpsrGuest>(
        get_parameter("memory_get_service").as_string());
    save_client_ = create_client<robot_ai_interfaces::srv::SaveGpsrGuest>(
        get_parameter("memory_save_service").as_string());
    profile_client_ = create_client<robot_ai_interfaces::srv::GetProfile>(
        get_parameter("profile_get_service").as_string());
    if (!get_client_->wait_for_service(
            std::chrono::duration<double>(memory_timeout_sec_)) ||
        !save_client_->wait_for_service(
            std::chrono::duration<double>(memory_timeout_sec_))) {
      release_resources();
      return CallbackReturn::FAILURE;
    }
    action_server_ = rclcpp_action::create_server<Action>(
        this, get_parameter("action_name").as_string(),
        std::bind(&ExtractPersonInfoServer::admit, this, std::placeholders::_1,
                  std::placeholders::_2),
        [](const std::shared_ptr<GoalHandle>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        std::bind(&ExtractPersonInfoServer::execute_async, this,
                  std::placeholders::_1));
    return CallbackReturn::SUCCESS;
  }

  // Special function: Enable action admission only after lifecycle activation.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    active_.store(true);
    return CallbackReturn::SUCCESS;
  }

  // Special function: Close action admission before lifecycle cleanup.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    active_.store(false);
    return busy_.load() ? CallbackReturn::FAILURE : CallbackReturn::SUCCESS;
  }

  // Special function: Release extraction resources only when no goal owns them.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    if (busy_.load()) return CallbackReturn::FAILURE;
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  // Special function: Fail closed during process shutdown.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    active_.store(false);
    release_resources();
    return CallbackReturn::SUCCESS;
  }

 private:
  // Special function: Cache the newest canonical observation for each active track.
  void observe(
      const robot_ai_interfaces::msg::TrackedPersonAppearanceArray::SharedPtr message) {
    if (!active_.load()) return;
    const auto received = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(observation_mutex_);
    for (const auto &person : message->people) {
      const auto key =
          person.tracking_session_id + ":" + std::to_string(person.track_id);
      latest_people_[key] = {person, received};
    }
  }

  // Special function: Reject invalid references, unsupported fields, inactive state,
  // and overlap.
  rclcpp_action::GoalResponse admit(const rclcpp_action::GoalUUID &,
                                    const std::shared_ptr<const Action::Goal> goal) {
    if (!active_.load() || busy_.exchange(true))
      return rclcpp_action::GoalResponse::REJECT;
    const auto person_ref = parse_json_object(goal->person_ref_json);
    const std::set<std::string> supported{
        "name",        "age",        "gender",    "favorite_drink", "hometown",
        "shirt_color", "glasses",    "hat",       "face_mask",      "pose",
        "gesture",     "profile_id", "person_id", "track_id"};
    bool valid = !goal->memory_id.empty() || !person_ref.empty();
    for (const auto &field : goal->fields)
      valid = valid && supported.count(normalize_field(field)) != 0U;
    if (!valid) {
      busy_.store(false);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Special function: Run blocking service composition outside executor callbacks.
  void execute_async(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  // Special function: Load, enrich, validate, and persist one guest record.
  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto result = std::make_shared<Action::Result>();
    const auto goal = goal_handle->get_goal();
    if (goal_handle->is_canceling()) {
      result->outcome = "canceled";
      goal_handle->canceled(result);
      busy_.store(false);
      return;
    }
    GuestRecord record;
    Json direct_ref = parse_json_object(goal->person_ref_json);
    if (!goal->memory_id.empty()) {
      auto request =
          std::make_shared<robot_ai_interfaces::srv::GetGpsrGuest::Request>();
      request->memory_id = goal->memory_id;
      auto future = get_client_->async_send_request(request);
      if (future.wait_for(std::chrono::duration<double>(memory_timeout_sec_)) !=
          std::future_status::ready) {
        finish_failure(goal_handle, result, "failed",
                       "GPSR guest-memory get timed out.");
        return;
      }
      const auto response = future.get();
      if (!response->success && direct_ref.empty()) {
        finish_failure(goal_handle, result, "not_found", response->message);
        return;
      }
      if (response->success) record = response->record;
    }
    if (record.person_ref_json.empty()) record.person_ref_json = direct_ref.dump();
    if (record.memory_id.empty()) record.memory_id = goal->memory_id;
    record.command_id = goal->command_id.empty() ? record.command_id : goal->command_id;
    record.source_step_id =
        goal->source_step_id.empty() ? record.source_step_id : goal->source_step_id;
    auto person_ref = parse_json_object(record.person_ref_json);
    if (person_ref.empty()) person_ref = direct_ref;
    auto info = parse_json_object(record.guest_info_json);
    merge_profile(info, record.profile);
    if (person_ref.contains("appearance") && person_ref["appearance"].is_object()) {
      for (const auto &item : person_ref["appearance"].items()) {
        if (item.key().find("_confidence") == std::string::npos &&
            item.value() != "unknown")
          info[item.key()] = item.value();
      }
    }
    for (const auto *field :
         {"pose", "gesture", "profile_id", "person_id", "track_id"}) {
      if (person_ref.contains(field) && !person_ref[field].is_null())
        info[field] = person_ref[field];
    }
    refresh_live(record, person_ref, info);
    refresh_profile(record, person_ref, info);
    std::vector<std::string> fields;
    for (const auto &field : goal->fields) fields.push_back(normalize_field(field));
    const std::set<std::string> all_fields{
        "name",        "age",        "gender",    "favorite_drink", "hometown",
        "shirt_color", "glasses",    "hat",       "face_mask",      "pose",
        "gesture",     "profile_id", "person_id", "track_id"};
    const auto requested =
        fields.empty() ? std::vector<std::string>(all_fields.begin(), all_fields.end())
                       : fields;
    for (const auto &field : requested) {
      if (!info.contains(field) || info[field].is_null() || info[field] == "" ||
          info[field] == "unknown" || info[field] == 0) {
        result->missing_fields.push_back(field);
      }
    }
    const bool semantically_complete =
        fields.empty() ? !info.empty() : result->missing_fields.empty();
    record.guest_info_json = info.dump();
    auto save_request =
        std::make_shared<robot_ai_interfaces::srv::SaveGpsrGuest::Request>();
    save_request->record = record;
    auto save_future = save_client_->async_send_request(save_request);
    if (save_future.wait_for(std::chrono::duration<double>(memory_timeout_sec_)) !=
        std::future_status::ready) {
      finish_failure(goal_handle, result, "failed",
                     "GPSR guest-memory save timed out.");
      return;
    }
    const auto save_response = save_future.get();
    if (!save_response->success) {
      finish_failure(goal_handle, result, "failed", save_response->message);
      return;
    }
    result->success = semantically_complete;
    result->outcome = semantically_complete ? "success" : "not_found";
    result->memory_id = save_response->record.memory_id;
    result->memory_revision = save_response->record.revision;
    result->info_json = info.dump();
    result->message = semantically_complete
                          ? "Requested guest information extracted and stored."
                          : "Guest information is incomplete.";
    if (semantically_complete)
      goal_handle->succeed(result);
    else
      goal_handle->abort(result);
    busy_.store(false);
  }

  // Special function: Overlay a fresh exact-track observation when available.
  void refresh_live(GuestRecord &record, const Json &person_ref, Json &info) {
    const auto key = person_ref.value("tracking_session_id", std::string()) + ":" +
                     std::to_string(person_ref.value("track_id", 0));
    std::lock_guard<std::mutex> lock(observation_mutex_);
    const auto iterator = latest_people_.find(key);
    if (iterator == latest_people_.end() ||
        std::chrono::steady_clock::now() - iterator->second.second >
            std::chrono::duration<double>(live_stale_timeout_sec_))
      return;
    const auto &person = iterator->second.first;
    record.profile = person.profile;
    if (!person.pose.empty() && person.pose != "unknown") info["pose"] = person.pose;
    if (!person.gesture.empty() && person.gesture != "unknown")
      info["gesture"] = person.gesture;
    if (!person.profile_id.empty()) info["profile_id"] = person.profile_id;
    info["person_id"] = person.person_id;
    info["track_id"] = person.track_id;
    merge_profile(info, person.profile);
  }

  // Special function: Refresh registered semantic fields when the optional profile
  // service responds.
  void refresh_profile(GuestRecord &record, const Json &person_ref, Json &info) {
    const auto profile_id = !record.profile.profile_id.empty()
                                ? record.profile.profile_id
                                : person_ref.value("profile_id", std::string());
    if (profile_id.empty() || !profile_client_->service_is_ready()) return;
    auto request = std::make_shared<robot_ai_interfaces::srv::GetProfile::Request>();
    request->profile_id = profile_id;
    auto future = profile_client_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(memory_timeout_sec_)) !=
        std::future_status::ready)
      return;
    const auto response = future.get();
    if (!response->success) return;
    record.profile = response->profile;
    merge_profile(info, record.profile);
  }

  // Special function: Complete an unsuccessful goal and release single-goal admission.
  void finish_failure(const std::shared_ptr<GoalHandle> &goal_handle,
                      const std::shared_ptr<Action::Result> &result,
                      const std::string &outcome, const std::string &message) {
    result->outcome = outcome;
    result->message = message;
    goal_handle->abort(result);
    busy_.store(false);
  }

  // Special function: Destroy configured handles and clear volatile live observations.
  void release_resources() {
    action_server_.reset();
    subscription_.reset();
    get_client_.reset();
    save_client_.reset();
    profile_client_.reset();
    std::lock_guard<std::mutex> lock(observation_mutex_);
    latest_people_.clear();
    busy_.store(false);
  }

  std::atomic_bool active_{false};
  std::atomic_bool busy_{false};
  double memory_timeout_sec_{2.0};
  double live_stale_timeout_sec_{1.0};
  std::mutex observation_mutex_;
  std::map<std::string, std::pair<robot_ai_interfaces::msg::TrackedPersonAppearance,
                                  std::chrono::steady_clock::time_point>>
      latest_people_;
  rclcpp::Subscription<
      robot_ai_interfaces::msg::TrackedPersonAppearanceArray>::SharedPtr subscription_;
  rclcpp::Client<robot_ai_interfaces::srv::GetGpsrGuest>::SharedPtr get_client_;
  rclcpp::Client<robot_ai_interfaces::srv::SaveGpsrGuest>::SharedPtr save_client_;
  rclcpp::Client<robot_ai_interfaces::srv::GetProfile>::SharedPtr profile_client_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
};

}  // namespace robot_yasmin_tasks

// Section: Process startup and lifecycle action-server execution.
int main(int argc, char **argv) {
  // Section: Four executor threads keep service responses live during action execution.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_yasmin_tasks::ExtractPersonInfoServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
