#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <putrabot_interfaces/msg/object_detections.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <yaml-cpp/yaml.h>
#include <robot_ai_interfaces/action/find_object.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_yasmin_tasks {

using Action = robot_ai_interfaces::action::FindObject;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
using Detection = putrabot_interfaces::msg::ObjectDetection;
using DetectionArray = putrabot_interfaces::msg::ObjectDetections;
using Json = nlohmann::json;

// Special function: Normalize detector and planner symbols to the same vocabulary.
std::string clean_symbol(std::string value) {
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char character) {
                return !std::isspace(character);
              }));
  value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char character) {
                return !std::isspace(character);
              }).base(), value.end());
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    if (character == ' ' || character == '-') return '_';
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

class FindObjectServer final : public rclcpp_lifecycle::LifecycleNode {
 public:
  // Special function: Declare the complete Python-compatible parameter surface.
  FindObjectServer() : LifecycleNode("find_object_skill_server") {
    declare_parameter<std::string>("action_name", "/gpsr/find_object");
    declare_parameter<std::string>("detections_topic", "/perception/raw_object_detections");
    declare_parameter<bool>("enable_depth", true);
    declare_parameter<std::string>("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
    declare_parameter<std::string>("named_navigation_action", "/nav/navigate_named_pose");
    declare_parameter<std::string>("navigate_to_pose_action", "/navigate_to_pose");
    declare_parameter<std::string>("spin_action", "/spin");
    declare_parameter<std::string>("compute_path_action", "/compute_path_to_pose");
    declare_parameter<std::string>("global_costmap_service", "/global_costmap/get_costmap");
    declare_parameter<std::string>("target_status_topic", "/gpsr/find_object/target_status");
    declare_parameter<std::string>("detector_ready_topic", "/perception/object_detector_ready");
    declare_parameter<std::string>("search_config_file", "");
    declare_parameter<std::string>("object_knowledge_file", "");
    declare_parameter<double>("default_timeout_sec", 300.0);
    rclcpp::QoS qos(1);
    qos.reliable().transient_local();
    status_publisher_ = create_publisher<std_msgs::msg::String>(get_parameter("target_status_topic").as_string(), qos);
    publish_status("IDLE", nullptr, 0.0, false, -1);
  }

  // Special function: Allocate perception inputs and the action endpoint as one lifecycle unit.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    default_timeout_sec_ = std::max(0.1, get_parameter("default_timeout_sec").as_double());
    if (!load_object_knowledge()) return CallbackReturn::FAILURE;
    detection_subscription_ = create_subscription<DetectionArray>(
        get_parameter("detections_topic").as_string(), rclcpp::SensorDataQoS(),
        std::bind(&FindObjectServer::detections_callback, this, std::placeholders::_1));
    rclcpp::QoS ready_qos(1);
    ready_qos.reliable().transient_local();
    ready_subscription_ = create_subscription<std_msgs::msg::Bool>(
        get_parameter("detector_ready_topic").as_string(), ready_qos,
        std::bind(&FindObjectServer::ready_callback, this, std::placeholders::_1));
    action_server_ = rclcpp_action::create_server<Action>(
        this, get_parameter("action_name").as_string(),
        std::bind(&FindObjectServer::admit, this, std::placeholders::_1, std::placeholders::_2),
        [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        std::bind(&FindObjectServer::execute_async, this, std::placeholders::_1));
    return CallbackReturn::SUCCESS;
  }

  // Special function: Admit missions only while the lifecycle server is ACTIVE.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    active_.store(true);
    return CallbackReturn::SUCCESS;
  }

  // Special function: Stop admission and wake a waiting perception mission.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    active_.store(false);
    condition_.notify_all();
    return busy_.load() ? CallbackReturn::FAILURE : CallbackReturn::SUCCESS;
  }

  // Special function: Release configured resources only after mission ownership ends.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    if (busy_.load()) return CallbackReturn::FAILURE;
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  // Special function: Fail closed and discard volatile detector samples on shutdown.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    active_.store(false);
    condition_.notify_all();
    release_resources();
    return CallbackReturn::SUCCESS;
  }

 private:
  // Special function: Load canonical labels, aliases, categories, and wildcard targets.
  bool load_object_knowledge() {
    // Section: Resolve the installed knowledge file when launch does not override it.
    auto path = get_parameter("object_knowledge_file").as_string();
    if (path.empty()) {
      path = ament_index_cpp::get_package_share_directory("robot_yasmin_tasks") +
             "/config/object_knowledge.yaml";
    }
    object_aliases_.clear();
    category_aliases_.clear();
    category_labels_.clear();
    wildcards_.clear();
    try {
      const auto root = YAML::LoadFile(path);
      const auto objects = root["objects"];
      for (const auto &entry : objects) {
        const auto canonical = clean_symbol(entry.first.as<std::string>());
        const auto specification = entry.second;
        const auto label = clean_symbol(
            specification["detector_label"]
                ? specification["detector_label"].as<std::string>()
                : canonical);
        object_aliases_[canonical] = label;
        object_aliases_[label] = label;
        if (specification["aliases"]) {
          for (const auto &alias : specification["aliases"]) {
            object_aliases_[clean_symbol(alias.as<std::string>())] = label;
          }
        }
      }
      const auto categories = root["categories"];
      for (const auto &entry : categories) {
        const auto canonical = clean_symbol(entry.first.as<std::string>());
        category_aliases_[canonical] = canonical;
        if (entry.second["aliases"]) {
          for (const auto &alias : entry.second["aliases"]) {
            category_aliases_[clean_symbol(alias.as<std::string>())] = canonical;
          }
        }
        for (const auto &member : entry.second["members"]) {
          const auto member_name = clean_symbol(member.as<std::string>());
          const auto found = object_aliases_.find(member_name);
          category_labels_[canonical].insert(
              found == object_aliases_.end() ? member_name : found->second);
        }
      }
      if (root["wildcards"]) {
        for (const auto &wildcard : root["wildcards"]) {
          wildcards_.insert(clean_symbol(wildcard.as<std::string>()));
        }
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Unable to load object knowledge '%s': %s",
                   path.c_str(), error.what());
      return false;
    }
    return !object_aliases_.empty();
  }

  // Special function: Resolve one goal into the detector labels it may match.
  std::set<std::string> target_labels(const Action::Goal &goal,
                                      bool &wildcard) const {
    // Section: Exact-object aliases take precedence over category membership.
    wildcard = false;
    const auto object = clean_symbol(goal.object_name);
    if (!object.empty()) {
      if (wildcards_.count(object) != 0U) {
        wildcard = true;
        return {};
      }
      const auto found = object_aliases_.find(object);
      return found == object_aliases_.end() ? std::set<std::string>{}
                                            : std::set<std::string>{found->second};
    }
    const auto category = clean_symbol(goal.category);
    const auto alias = category_aliases_.find(category);
    if (alias == category_aliases_.end()) return {};
    const auto labels = category_labels_.find(alias->second);
    return labels == category_labels_.end() ? std::set<std::string>{}
                                            : labels->second;
  }

  // Special function: Retain one complete detector frame, including an empty frame.
  void detections_callback(const DetectionArray::SharedPtr message) {
    if (!active_.load()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    detections_ = message->detections;
    ++sequence_;
    condition_.notify_all();
  }

  // Special function: Track the detector's transient-local live-inference readiness.
  void ready_callback(const std_msgs::msg::Bool::SharedPtr message) {
    detector_ready_.store(message->data);
    condition_.notify_all();
  }

  // Special function: Reject malformed, inactive, or concurrent object missions.
  rclcpp_action::GoalResponse admit(const rclcpp_action::GoalUUID &,
                                    const std::shared_ptr<const Action::Goal> goal) {
    const auto scope = goal->search_scope.empty() ? (goal->location.empty() ? "GLOBAL" : "STRICT_LOCATION")
                                                   : goal->search_scope;
    if ((!goal->object_name.empty() || !goal->category.empty()) &&
        (scope == "GLOBAL" || (scope == "STRICT_LOCATION" && !goal->location.empty())) &&
        active_.load() && !busy_.exchange(true)) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Special function: Move the bounded perception mission off executor callback threads.
  void execute_async(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  // Special function: Confirm the selected semantic target across a sliding frame window.
  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();
    bool wildcard = false;
    const auto labels = target_labels(*goal, wildcard);
    const auto timeout = goal->timeout_sec > 0.0F ? goal->timeout_sec : default_timeout_sec_;
    const auto started = std::chrono::steady_clock::now();
    const auto deadline = started + std::chrono::duration<double>(timeout);
    publish_status("GOAL_ACCEPTED", goal.get(), 0.0, false, -1);
    std::uint64_t cursor = 0;
    unsigned hits = 0;
    std::optional<Detection> best;
    while (std::chrono::steady_clock::now() < deadline && active_.load() && !goal_handle->is_canceling()) {
      std::vector<Detection> frame;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(lock, std::chrono::milliseconds(100), [this, cursor]() {
          return sequence_ > cursor || !active_.load();
        });
        if (sequence_ <= cursor) continue;
        cursor = sequence_;
        frame = detections_;
      }
      std::optional<Detection> frame_best;
      for (const auto &detection : frame) {
        const auto name = clean_symbol(detection.name);
        const bool semantic_match = wildcard || labels.count(name) != 0U;
        if (semantic_match && detection.confidence >= 0.50F &&
            (!frame_best || detection.confidence > frame_best->confidence)) frame_best = detection;
      }
      if (frame_best) {
        ++hits;
        best = frame_best;
      } else {
        hits = 0;
      }
      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      auto feedback = std::make_shared<Action::Feedback>();
      feedback->phase = best ? "VERIFYING" : "SEARCHING";
      feedback->current_room = goal->location;
      feedback->candidate_confidence = best ? best->confidence : 0.0F;
      feedback->elapsed_sec = static_cast<float>(elapsed);
      goal_handle->publish_feedback(feedback);
      publish_status(feedback->phase, goal.get(), feedback->candidate_confidence, false, -1);
      if (hits >= 3U) break;
    }
    auto result = std::make_shared<Action::Result>();
    if (goal_handle->is_canceling() || !active_.load()) {
      result->result_code = Action::Result::CANCELED;
      result->message = "FindObject mission canceled.";
      publish_status("CANCELED", goal.get(), best ? best->confidence : 0.0, true, result->result_code);
      goal_handle->canceled(result);
    } else if (!detector_ready_.load() || cursor == 0U) {
      result->result_code = Action::Result::PERCEPTION_FAILED;
      result->message = "Object perception did not provide ready, fresh detection frames.";
      publish_status("PERCEPTION_FAILED", goal.get(), 0.0, true, result->result_code);
      goal_handle->abort(result);
    } else if (!best || hits < 3U) {
      result->result_code = Action::Result::NOT_FOUND;
      result->message = "Target was not confirmed before the search deadline.";
      publish_status("NOT_FOUND", goal.get(), best ? best->confidence : 0.0, true, result->result_code);
      goal_handle->succeed(result);
    } else {
      result->found = true;
      result->approached = false;
      result->result_code = goal->approach_target ? Action::Result::FOUND_UNREACHABLE : Action::Result::SUCCESS;
      result->object_id = clean_symbol(best->name) + "_" + std::to_string(static_cast<int>(best->center_u)) + "_" +
                          std::to_string(static_cast<int>(best->center_v));
      result->found_room = goal->location;
      result->message = goal->approach_target
                            ? "Target verified, but approach geometry is not available from RGB detections alone."
                            : "Target verified from the live object detector.";
      publish_status(goal->approach_target ? "FOUND_UNREACHABLE" : "SUCCESS", goal.get(), best->confidence,
                     true, result->result_code, result->object_id);
      goal_handle->succeed(result);
    }
    busy_.store(false);
  }

  // Special function: Publish the transient target-state JSON contract for observers.
  void publish_status(const std::string &phase, const Action::Goal *goal, const double confidence,
                      const bool terminal, const int result_code, const std::string &object_id = "") {
    Json payload{{"phase", phase},
                 {"target_name", goal ? clean_symbol(goal->object_name) : ""},
                 {"category", goal ? clean_symbol(goal->category) : ""},
                 {"location", goal ? clean_symbol(goal->location) : ""},
                 {"detected", confidence > 0.0},
                 {"verified", terminal && confidence > 0.0},
                 {"found", terminal && (result_code == 0 || result_code == 2 || result_code == 3)},
                 {"confidence", confidence},
                 {"object_id", object_id},
                 {"found_room", terminal && goal ? clean_symbol(goal->location) : ""},
                 {"found_viewpoint", ""},
                 {"result_code", result_code},
                 {"terminal", terminal}};
    std_msgs::msg::String message;
    message.data = payload.dump();
    status_publisher_->publish(message);
  }

  // Special function: Reset every lifecycle-owned endpoint and volatile sample.
  void release_resources() {
    action_server_.reset();
    detection_subscription_.reset();
    ready_subscription_.reset();
    std::lock_guard<std::mutex> lock(mutex_);
    detections_.clear();
    sequence_ = 0;
    detector_ready_.store(false);
    busy_.store(false);
  }

  std::atomic_bool active_{false};
  std::atomic_bool busy_{false};
  std::atomic_bool detector_ready_{false};
  // Section: Immutable semantic indices loaded at lifecycle configuration time.
  std::map<std::string, std::string> object_aliases_;
  std::map<std::string, std::string> category_aliases_;
  std::map<std::string, std::set<std::string>> category_labels_;
  std::set<std::string> wildcards_;
  double default_timeout_sec_{300.0};
  std::mutex mutex_;
  std::condition_variable condition_;
  std::uint64_t sequence_{0};
  std::vector<Detection> detections_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<DetectionArray>::SharedPtr detection_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_subscription_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
};

}  // namespace robot_yasmin_tasks

int main(int argc, char **argv) {
  // Section: Four executor threads keep perception callbacks responsive during action work.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_yasmin_tasks::FindObjectServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
