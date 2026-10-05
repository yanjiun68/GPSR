// Section: Lifecycle interactive FIND_PERSON server backed by persistent appearance tracks.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cctype>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_ai_interfaces/action/find_person.hpp>
#include <robot_ai_interfaces/msg/tracked_person_appearance_array.hpp>
#include <set>
#include <string>
#include <thread>
#include <voice_interfaces/srv/say_text.hpp>
#include <vector>

namespace robot_yasmin_tasks {

// Section: Short aliases keep the action and observation policy readable.
using FindAction = robot_ai_interfaces::action::FindPerson;
using FindGoalHandle = rclcpp_action::ServerGoalHandle<FindAction>;
using Observation = robot_ai_interfaces::msg::TrackedPersonAppearance;
using ObservationArray = robot_ai_interfaces::msg::TrackedPersonAppearanceArray;
using Json = nlohmann::json;

// Special function: Normalize names, gestures, and selector values for exact matching.
std::string normalize(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char character) {
                   if (character == ' ' || character == '-') return '_';
                   return static_cast<char>(std::tolower(character));
                 });
  return value;
}

// Special function: Recognize all supported raised-hand gesture aliases.
bool is_raised_hand(const Observation &person, double confidence_threshold) {
  const auto gesture = normalize(person.gesture);
  const bool alias = gesture == "right_hand_raised" ||
                     gesture == "left_hand_raised" ||
                     gesture == "raising_right_arm" ||
                     gesture == "raising_left_arm" || gesture == "hands_up";
  return alias && person.gesture_confidence >= confidence_threshold;
}

// Special function: Serialize one track identity without depending on a new ROS message.
std::string track_key(const Observation &person) {
  return person.tracking_session_id + ":" + std::to_string(person.track_id);
}

// Special function: Convert one tracked observation to a transport-safe JSON record.
Json observation_json(const Observation &person) {
  return Json{{"tracking_session_id", person.tracking_session_id},
              {"track_id", person.track_id},
              {"person_id", person.person_id},
              {"profile_id", person.profile_id},
              {"person_uuid", person.person_uuid},
              {"pose", person.pose},
              {"pose_confidence", person.pose_confidence},
              {"gesture", person.gesture},
              {"gesture_confidence", person.gesture_confidence},
              {"bbox_xyxy", person.bbox_xyxy}};
}

// Special function: Evaluate the deterministic attributes already emitted by the tracker.
bool matches_attribute(const Observation &person, const Json &attribute) {
  const auto type = normalize(attribute.value("type", std::string{}));
  auto value = normalize(attribute.value("value", std::string{}));
  if (type == "pose") {
    if (value == "lying") value = "lying_down";
    return normalize(person.pose) == value && person.pose_confidence > 0.0F;
  }
  if (type == "gesture") {
    return normalize(person.gesture) == value && person.gesture_confidence > 0.0F;
  }
  if (type == "shirt_color" || type == "clothes_color") {
    return normalize(person.appearance.shirt_color) == value &&
           person.appearance.shirt_color_confidence > 0.0F;
  }
  if (type == "glasses") return person.appearance.glasses == 2U;
  if (type == "hat") return person.appearance.hat == 2U;
  if (type == "face_mask") return person.appearance.face_mask == 2U;
  return false;
}

class FindPersonServer final : public rclcpp_lifecycle::LifecycleNode {
 public:
  // Special function: Declare all bounded interactive-search configuration in one node.
  FindPersonServer() : LifecycleNode("find_person_skill_server") {
    declare_parameter<std::string>("action_name", "/gpsr/find_person");
    declare_parameter<std::string>("appearance_topic", "/appearance/tracked_people");
    declare_parameter<std::string>("tts_service", "/voice/tts/say");
    declare_parameter<double>("default_timeout_sec", 30.0);
    declare_parameter<double>("gesture_confidence_threshold", 0.55);
    declare_parameter<std::int64_t>("gesture_min_stable_frames", 3);
    declare_parameter<double>("tts_timeout_sec", 3.0);
  }

  // Special function: Allocate the action, tracked-observation, and TTS interfaces.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    default_timeout_sec_ = std::max(0.1, get_parameter("default_timeout_sec").as_double());
    gesture_confidence_threshold_ = std::clamp(
        get_parameter("gesture_confidence_threshold").as_double(), 0.0, 1.0);
    gesture_min_stable_frames_ = static_cast<std::uint32_t>(std::max<std::int64_t>(
        1, get_parameter("gesture_min_stable_frames").as_int()));
    tts_timeout_sec_ = std::max(0.1, get_parameter("tts_timeout_sec").as_double());
    subscription_ = create_subscription<ObservationArray>(
        get_parameter("appearance_topic").as_string(), 10,
        std::bind(&FindPersonServer::observe, this, std::placeholders::_1));
    tts_client_ = create_client<voice_interfaces::srv::SayText>(
        get_parameter("tts_service").as_string());
    action_server_ = rclcpp_action::create_server<FindAction>(
        this, get_parameter("action_name").as_string(),
        std::bind(&FindPersonServer::admit, this, std::placeholders::_1,
                  std::placeholders::_2),
        [](const std::shared_ptr<FindGoalHandle>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        std::bind(&FindPersonServer::execute_async, this, std::placeholders::_1));
    return CallbackReturn::SUCCESS;
  }

  // Special function: Permit goals only after lifecycle activation.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    active_.store(true);
    return CallbackReturn::SUCCESS;
  }

  // Special function: Stop admission and wake all bounded waits.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    active_.store(false);
    condition_.notify_all();
    return busy_.load() ? CallbackReturn::FAILURE : CallbackReturn::SUCCESS;
  }

  // Special function: Release interfaces only after the action worker is idle.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    if (busy_.load()) return CallbackReturn::FAILURE;
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  // Special function: Fail closed during shutdown and release buffered observations.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    active_.store(false);
    condition_.notify_all();
    release_resources();
    return CallbackReturn::SUCCESS;
  }

 private:
  // Special class: Release exclusive mission ownership on every worker exit path.
  class BusyResetGuard final {
   public:
    // Special function: Retain the lifecycle server that owns the atomic mission lock.
    explicit BusyResetGuard(FindPersonServer *server) : server_(server) {}

    // Special function: Preserve action availability after success, failure, or exception.
    ~BusyResetGuard() noexcept { server_->busy_.store(false); }

   private:
    // Section: The action worker cannot outlive its owning lifecycle node.
    FindPersonServer *server_;
  };

  // Special function: Cache fresh frames and wake the action worker without blocking it.
  void observe(const ObservationArray::SharedPtr message) {
    if (!active_.load()) return;
    std::lock_guard<std::mutex> lock(observation_mutex_);
    ++sequence_;
    ++frame_count_;
    last_frame_time_ = std::chrono::steady_clock::now();
    frames_.push_back({sequence_, *message});
    if (frames_.size() > 120U) frames_.erase(frames_.begin());
    condition_.notify_all();
  }

  // Special function: Validate a name-only or name-plus-visible-attribute query.
  rclcpp_action::GoalResponse admit(
      const rclcpp_action::GoalUUID &, const std::shared_ptr<const FindAction::Goal> goal) {
    // Section: Report the actual admission failure rather than silently rejecting clients.
    if (!active_.load()) {
      RCLCPP_WARN(get_logger(), "Rejected FindPerson goal: lifecycle server is inactive.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected_busy = false;
    if (!busy_.compare_exchange_strong(expected_busy, true)) {
      RCLCPP_WARN(get_logger(), "Rejected FindPerson goal: another mission is running.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->person_name.empty()) {
      RCLCPP_WARN(get_logger(), "Rejected FindPerson goal: person_name is empty.");
      busy_.store(false);
      return rclcpp_action::GoalResponse::REJECT;
    }
    try {
      if (!goal->description.empty()) {
        const auto query = Json::parse(goal->description);
        const auto attributes = query.value("attributes", Json::array());
        if (!attributes.is_array()) throw std::runtime_error("attributes must be an array");
        for (const auto &attribute : attributes) {
          const auto type = normalize(attribute.value("type", std::string{}));
          const std::set<std::string> supported{
              "pose", "gesture", "shirt_color", "clothes_color", "glasses", "hat", "face_mask"};
          if (supported.count(type) == 0U) throw std::runtime_error("unsupported attribute");
        }
      }
    } catch (const std::exception &) {
      RCLCPP_WARN(get_logger(), "Rejected FindPerson goal: description JSON is invalid.");
      busy_.store(false);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Special function: Move blocking TTS and observation work off executor callbacks.
  void execute_async(const std::shared_ptr<FindGoalHandle> goal_handle) {
    std::thread([this, goal_handle]() {
      // Section: Convert unexpected worker failures into a terminal action response.
      try {
        execute(goal_handle);
      } catch (const std::exception &error) {
        RCLCPP_ERROR(get_logger(), "FindPerson worker failed: %s", error.what());
        auto result = std::make_shared<FindAction::Result>();
        result->message = std::string("INTERNAL_ERROR: ") + error.what();
        goal_handle->abort(result);
        busy_.store(false);
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "FindPerson worker failed with an unknown exception.");
        auto result = std::make_shared<FindAction::Result>();
        result->message = "INTERNAL_ERROR: unknown FindPerson worker exception.";
        goal_handle->abort(result);
        busy_.store(false);
      }
    }).detach();
  }

  // Special function: Speak the request and verify one stable new raised-hand transition.
  void execute(const std::shared_ptr<FindGoalHandle> goal_handle) {
    // Section: Ensure every early return releases the exclusive FIND_PERSON mission lock.
    const BusyResetGuard busy_reset(this);
    const auto goal = goal_handle->get_goal();
    const auto frames_at_goal_start = frame_count_.load();
    const auto timeout = goal->timeout_sec > 0.0F ? goal->timeout_sec : default_timeout_sec_;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(std::max(0.1, timeout)));
    Json query = Json::object();
    try {
      query = goal->description.empty() ? Json::object() : Json::parse(goal->description);
    } catch (const std::exception &) {
      finish_failure(goal_handle, "INVALID_QUERY: malformed person query", false);
      return;
    }
    const auto attributes = query.value("attributes", Json::array());
    std::map<std::string, bool> baseline;
    std::uint64_t cursor = 0;
    ObservationArray first_frame;
    if (!wait_frame(cursor, first_frame, deadline, goal_handle)) {
      finish_failure(goal_handle, perception_failure_message(frames_at_goal_start), true);
      return;
    }
    for (const auto &person : first_frame.people) baseline[track_key(person)] = is_raised_hand(person, gesture_confidence_threshold_);
    if (!say("" + goal->person_name + ", please raise your hand.")) {
      finish_failure(goal_handle, "PERCEPTION_FAILED: TTS request failed", true);
      return;
    }
    std::map<std::string, std::uint32_t> stable;
    std::optional<Observation> match;
    bool saw_people = !first_frame.people.empty();
    while (std::chrono::steady_clock::now() < deadline && active_.load() &&
           !goal_handle->is_canceling()) {
      ObservationArray frame;
      if (!wait_frame(cursor, frame, deadline, goal_handle)) break;
      saw_people = saw_people || !frame.people.empty();
      std::set<std::string> observed;
      for (const auto &person : frame.people) {
        const auto key = track_key(person);
        observed.insert(key);
        const bool was_raised = baseline.count(key) != 0U && baseline[key];
        bool attributes_match = true;
        for (const auto &attribute : attributes) attributes_match = attributes_match && matches_attribute(person, attribute);
        const bool transitioned = !was_raised && is_raised_hand(person, gesture_confidence_threshold_);
        if (attributes_match && transitioned) {
          if (++stable[key] >= gesture_min_stable_frames_) {
            match = person;
            break;
          }
        } else {
          stable[key] = 0;
        }
      }
      for (auto &[key, count] : stable) if (observed.count(key) == 0U) count = 0;
      auto feedback = std::make_shared<FindAction::Feedback>();
      feedback->status = match ? "matched" : (saw_people ? "waiting_for_raised_hand" : "rescanning_for_people");
      goal_handle->publish_feedback(feedback);
      if (match) break;
    }
    if (goal_handle->is_canceling()) {
      finish_canceled(goal_handle);
    } else if (match) {
      auto result = std::make_shared<FindAction::Result>();
      result->found = true;
      result->person_id = track_key(*match);
      result->person_pose.header = match->header;
      result->message = "Person verified by a stable raised-hand response.";
      goal_handle->succeed(result);
    } else {
      finish_failure(goal_handle,
                    (saw_people ? "GESTURE_TIMEOUT: no stable raised-hand response"
                                : "PERSON_NOT_FOUND: no person was detected after the scan"),
                    true);
    }
    busy_.store(false);
  }

  // Special function: Wait for a frame newer than the supplied cursor and preserve cancellation.
  bool wait_frame(std::uint64_t &cursor, ObservationArray &frame,
                  const std::chrono::steady_clock::time_point &deadline,
                  const std::shared_ptr<FindGoalHandle> &goal_handle) {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    condition_.wait_until(lock, deadline, [this, cursor, &goal_handle]() {
      return sequence_ > cursor || !active_.load() || goal_handle->is_canceling();
    });
    if (!active_.load() || goal_handle->is_canceling()) return false;
    for (const auto &item : frames_) {
      if (item.first > cursor) {
        cursor = item.first;
        frame = item.second;
        return true;
      }
    }
    return false;
  }

  // Special function: Report exact tracked-person stream evidence in an action result.
  std::string perception_failure_message(const std::uint64_t frames_at_goal_start) {
    // Section: Snapshot the last-frame age under the observation lock.
    std::optional<std::chrono::steady_clock::time_point> last_frame;
    {
      std::lock_guard<std::mutex> lock(observation_mutex_);
      last_frame = last_frame_time_;
    }
    const auto frames_during_goal = frame_count_.load() - frames_at_goal_start;
    const auto publishers = subscription_ ? subscription_->get_publisher_count() : 0U;
    const auto age = last_frame
        ? std::to_string(std::chrono::duration<double>(
              std::chrono::steady_clock::now() - *last_frame).count())
        : "none";
    const auto reason = publishers == 0U ? "NO_APPEARANCE_PUBLISHER"
                        : frames_during_goal == 0U ? "NO_FRESH_TRACKED_PERSON_FRAME"
                                                    : "TRACKED_PERSON_STREAM_STALE";
    return "PERCEPTION_FAILED: " + std::string(reason) + "; appearance_topic=" +
           get_parameter("appearance_topic").as_string() + "; publishers=" +
           std::to_string(publishers) + "; frames_during_goal=" +
           std::to_string(frames_during_goal) + "; last_frame_age_sec=" + age + ".";
  }

  // Special function: Issue one bounded TTS request; speech never controls hardware.
  bool say(const std::string &text) {
    if (!tts_client_ || !tts_client_->wait_for_service(std::chrono::duration<double>(tts_timeout_sec_))) return false;
    auto request = std::make_shared<voice_interfaces::srv::SayText::Request>();
    request->text = text;
    auto future = tts_client_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(tts_timeout_sec_)) != std::future_status::ready) return false;
    return future.get()->success;
  }

  // Special function: Publish one standardized canceled result.
  void finish_canceled(const std::shared_ptr<FindGoalHandle> &goal_handle) {
    auto result = std::make_shared<FindAction::Result>();
    result->message = "FIND_PERSON canceled.";
    goal_handle->canceled(result);
    busy_.store(false);
  }

  // Special function: Publish one standardized recoverable failure result.
  void finish_failure(const std::shared_ptr<FindGoalHandle> &goal_handle,
                     const std::string &message, bool abort) {
    auto result = std::make_shared<FindAction::Result>();
    result->message = message;
    (void)abort;
    goal_handle->abort(result);
    busy_.store(false);
  }

  // Special function: Release every configured interface and buffered frame.
  void release_resources() {
    action_server_.reset();
    subscription_.reset();
    tts_client_.reset();
    std::lock_guard<std::mutex> lock(observation_mutex_);
    frames_.clear();
    sequence_ = 0;
    frame_count_.store(0);
    last_frame_time_.reset();
    busy_.store(false);
  }

  std::atomic_bool active_{false};
  std::atomic_bool busy_{false};
  double default_timeout_sec_{30.0};
  double gesture_confidence_threshold_{0.55};
  std::uint32_t gesture_min_stable_frames_{3};
  double tts_timeout_sec_{3.0};
  std::mutex observation_mutex_;
  std::condition_variable condition_;
  std::uint64_t sequence_{0};
  std::atomic_uint64_t frame_count_{0};
  std::optional<std::chrono::steady_clock::time_point> last_frame_time_;
  std::vector<std::pair<std::uint64_t, ObservationArray>> frames_;
  rclcpp::Subscription<ObservationArray>::SharedPtr subscription_;
  rclcpp::Client<voice_interfaces::srv::SayText>::SharedPtr tts_client_;
  rclcpp_action::Server<FindAction>::SharedPtr action_server_;
};

}  // namespace robot_yasmin_tasks

// Section: Start the lifecycle-managed interactive FIND_PERSON action server.
int main(int argc, char **argv) {
  // Section: A multi-threaded executor keeps tracked frames and TTS futures live.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_yasmin_tasks::FindPersonServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
