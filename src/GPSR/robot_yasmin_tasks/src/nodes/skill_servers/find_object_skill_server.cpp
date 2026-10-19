// Section: Lifecycle object-search server coordinating perception, navigation, depth,
// and target reporting.
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <future>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <optional>
#include <putrabot_interfaces/msg/object_detections.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_ai_interfaces/action/find_object.hpp>
#include <robot_ai_interfaces/action/navigate_named_pose.hpp>
#include <set>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

namespace robot_yasmin_tasks {

using Action = robot_ai_interfaces::action::FindObject;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
using NavigateNamedPose = robot_ai_interfaces::action::NavigateNamedPose;
using Spin = nav2_msgs::action::Spin;
using Detection = putrabot_interfaces::msg::ObjectDetection;
using DetectionArray = putrabot_interfaces::msg::ObjectDetections;
using Json = nlohmann::json;

// Special function: Normalize detector and planner symbols to the same vocabulary.
// Section: Search-query normalization, perception readiness, navigation, and
// object-result handling.
std::string clean_symbol(std::string value) {
  value.erase(value.begin(),
              std::find_if(value.begin(), value.end(), [](unsigned char character) {
                return !std::isspace(character);
              }));
  value.erase(
      std::find_if(value.rbegin(), value.rend(),
                   [](unsigned char character) { return !std::isspace(character); })
          .base(),
      value.end());
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char character) {
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
    declare_parameter<std::string>("detections_topic",
                                   "/perception/raw_object_detections");
    declare_parameter<bool>("enable_depth", true);
    declare_parameter<std::string>("depth_topic",
                                   "/camera/camera/aligned_depth_to_color/image_raw");
    declare_parameter<std::string>("camera_info_topic",
                                   "/camera/camera/color/camera_info");
    declare_parameter<std::string>("named_navigation_action",
                                   "/nav/navigate_named_pose");
    declare_parameter<std::string>("navigate_to_pose_action", "/navigate_to_pose");
    declare_parameter<std::string>("spin_action", "/spin");
    declare_parameter<std::string>("compute_path_action", "/compute_path_to_pose");
    declare_parameter<std::string>("global_costmap_service",
                                   "/global_costmap/get_costmap");
    declare_parameter<std::string>("target_status_topic",
                                   "/gpsr/find_object/target_status");
    declare_parameter<std::string>("detector_ready_topic",
                                   "/perception/object_detector_ready");
    declare_parameter<std::string>("detector_query_topic",
                                   "/perception/object_detector_query");
    declare_parameter<std::string>("search_config_file", "");
    declare_parameter<bool>("enable_navigation", true);
    declare_parameter<double>("camera_horizontal_fov_deg", 69.4);
    declare_parameter<std::string>("object_knowledge_file", "");
    declare_parameter<std::string>("furniture_map_file", "");
    declare_parameter<double>("depth_max_age_sec", 0.75);
    declare_parameter<double>("depth_min_valid_ratio", 0.5);
    declare_parameter<double>("support_height_tolerance_m", 0.12);
    declare_parameter<double>("default_timeout_sec", 300.0);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    rclcpp::QoS qos(1);
    qos.reliable().transient_local();
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        get_parameter("target_status_topic").as_string(), qos);
    publish_status("IDLE", nullptr, 0.0, false, -1);
  }

  // Special function: Allocate perception inputs and the action endpoint as one
  // lifecycle unit.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    default_timeout_sec_ =
        std::max(0.1, get_parameter("default_timeout_sec").as_double());
    if (!load_object_knowledge()) return CallbackReturn::FAILURE;
    if (!load_search_policy()) return CallbackReturn::FAILURE;
    if (!load_furniture_map()) return CallbackReturn::FAILURE;
    detection_subscription_ = create_subscription<DetectionArray>(
        get_parameter("detections_topic").as_string(), rclcpp::SensorDataQoS(),
        std::bind(&FindObjectServer::detections_callback, this, std::placeholders::_1));
    rclcpp::QoS ready_qos(1);
    ready_qos.reliable().transient_local();
    ready_subscription_ = create_subscription<std_msgs::msg::Bool>(
        get_parameter("detector_ready_topic").as_string(), ready_qos,
        std::bind(&FindObjectServer::ready_callback, this, std::placeholders::_1));
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        get_parameter("depth_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::SharedPtr message) {
          std::lock_guard<std::mutex> lock(depth_mutex_); depth_image_ = message;
        });
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        get_parameter("camera_info_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::SharedPtr message) {
          std::lock_guard<std::mutex> lock(depth_mutex_); camera_info_ = message;
        });
    // Section: One active FIND_OBJECT goal publishes its typed Qwen query before
    // waiting for fresh frames.
    detector_query_publisher_ = create_publisher<std_msgs::msg::String>(
        get_parameter("detector_query_topic").as_string(), 10);
    // Section: Reuse the already-running named-waypoint and Nav2 spin servers.
    named_navigation_client_ = rclcpp_action::create_client<NavigateNamedPose>(
        this, get_parameter("named_navigation_action").as_string());
    spin_client_ = rclcpp_action::create_client<Spin>(
        this, get_parameter("spin_action").as_string());
    action_server_ = rclcpp_action::create_server<Action>(
        this, get_parameter("action_name").as_string(),
        std::bind(&FindObjectServer::admit, this, std::placeholders::_1,
                  std::placeholders::_2),
        [](const std::shared_ptr<GoalHandle>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
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
  // Special class: Release the one-mission admission lock on every worker exit path.
  class BusyResetGuard final {
   public:
    // Special function: Retain the server whose lock and status must be restored.
    explicit BusyResetGuard(FindObjectServer *server) : server_(server) {}

    // Special function: Clear the mission lock even during stack unwinding.
    ~BusyResetGuard() noexcept {
      // Section: A completed, canceled, or failed worker must never block later goals.
      server_->busy_.store(false);
      try {
        // Section: Replace stale constructor-only IDLE status with the current availability.
        server_->publish_status("IDLE", nullptr, 0.0, false, -1, "", "");
      } catch (...) {
        // Section: A status publication failure must not prevent lock cleanup.
      }
    }

   private:
    // Section: The owning lifecycle server remains alive while its worker thread runs.
    FindObjectServer *server_;
  };

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
        const auto label =
            clean_symbol(specification["detector_label"]
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

  // Special function: Load named viewpoints and relative scan headings from policy.
  bool load_search_policy() {
    // Section: Resolve the installed search policy when launch does not override it.
    auto path = get_parameter("search_config_file").as_string();
    if (path.empty()) {
      path = ament_index_cpp::get_package_share_directory("robot_yasmin_tasks") +
             "/config/find_object_search.yaml";
    }
    room_viewpoints_.clear();
    global_viewpoints_.clear();
    scan_angles_deg_.clear();
    try {
      const auto root = YAML::LoadFile(path);
      const auto rooms = root["rooms"];
      for (const auto &entry : rooms) {
        std::vector<std::string> viewpoints;
        for (const auto &viewpoint : entry.second["viewpoints"])
          viewpoints.push_back(clean_symbol(viewpoint.as<std::string>()));
        // Section: Preserve the policy's YAML order for GLOBAL physical search.
        global_viewpoints_.insert(global_viewpoints_.end(), viewpoints.begin(),
                                  viewpoints.end());
        room_viewpoints_[clean_symbol(entry.first.as<std::string>())] = viewpoints;
      }
      const auto angles = root["scan"]["fast_angles_deg"];
      for (const auto &angle : angles) scan_angles_deg_.push_back(angle.as<double>());
      if (scan_angles_deg_.empty()) scan_angles_deg_.push_back(0.0);
      navigation_server_timeout_sec_ = root["navigation"]["server_timeout_sec"]
                                            ? root["navigation"]["server_timeout_sec"].as<double>() : 5.0;
      navigation_result_timeout_sec_ = root["navigation"]["result_timeout_sec"]
                                            ? root["navigation"]["result_timeout_sec"].as<double>() : 120.0;
      scan_action_timeout_sec_ = root["scan"]["action_timeout_sec"]
                                     ? root["scan"]["action_timeout_sec"].as<double>() : 20.0;
      confirmation_timeout_sec_ = root["verification"]["confirmation_timeout_sec"]
                                      ? root["verification"]["confirmation_timeout_sec"].as<double>() : 3.0;
      local_scan_step_deg_ = root["scan"]["local_step_deg"]
                                  ? root["scan"]["local_step_deg"].as<double>() : 15.0;
      local_scan_max_yaw_deg_ = root["scan"]["local_max_yaw_deg"]
                                     ? root["scan"]["local_max_yaw_deg"].as<double>() : 1080.0;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Unable to load FindObject search policy '%s': %s",
                   path.c_str(), error.what());
      return false;
    }
    return !room_viewpoints_.empty();
  }

  // Special function: Resolve strict or global object-search waypoints from policy.
  std::vector<std::string> search_viewpoints(const Action::Goal &goal) const {
    // Section: A strict location selects only that configured room's viewpoints.
    if (goal.search_scope == "STRICT_LOCATION" ||
        (goal.search_scope.empty() && !goal.location.empty())) {
      const auto found = room_viewpoints_.find(clean_symbol(goal.location));
      return found == room_viewpoints_.end() ? std::vector<std::string>{} : found->second;
    }
    // Section: Global search preserves the configured room order.
    return global_viewpoints_;
  }

  // Special function: Execute one existing named-waypoint action with bounded waits.
  bool navigate_to_viewpoint(const std::string &viewpoint) {
    // Section: Fail clearly when the external navigation stack is unavailable.
    if (!named_navigation_client_ || !named_navigation_client_->wait_for_action_server(
                                        std::chrono::duration<double>(navigation_server_timeout_sec_)))
      return false;
    NavigateNamedPose::Goal navigation_goal;
    navigation_goal.target_name = viewpoint;
    auto goal_future = named_navigation_client_->async_send_goal(navigation_goal);
    if (goal_future.wait_for(std::chrono::duration<double>(navigation_server_timeout_sec_)) != std::future_status::ready)
      return false;
    const auto handle = goal_future.get();
    if (!handle) return false;
    auto result_future = named_navigation_client_->async_get_result(handle);
    if (result_future.wait_for(std::chrono::duration<double>(navigation_result_timeout_sec_)) != std::future_status::ready) {
      named_navigation_client_->async_cancel_goal(handle);
      return false;
    }
    const auto result = result_future.get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
           result.result->success;
  }

  // Special function: Rotate the robot through one relative observation heading.
  bool spin_by(const double yaw_radians) {
    // Section: A zero heading is already covered by the arrival orientation.
    if (std::abs(yaw_radians) < 1e-6) return true;
    if (!spin_client_ || !spin_client_->wait_for_action_server(
                           std::chrono::duration<double>(navigation_server_timeout_sec_)))
      return false;
    Spin::Goal spin_goal;
    spin_goal.target_yaw = static_cast<float>(yaw_radians);
    spin_goal.time_allowance = rclcpp::Duration::from_seconds(scan_action_timeout_sec_);
    auto goal_future = spin_client_->async_send_goal(spin_goal);
    if (goal_future.wait_for(std::chrono::duration<double>(navigation_server_timeout_sec_)) != std::future_status::ready)
      return false;
    const auto handle = goal_future.get();
    if (!handle) return false;
    auto result_future = spin_client_->async_get_result(handle);
    return result_future.wait_for(std::chrono::duration<double>(scan_action_timeout_sec_ + 5.0)) == std::future_status::ready &&
           result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  // Special function: Generate bounded local-search headings for up to three
  // complete in-place sweeps.
  std::vector<double> local_scan_headings() const {
    // Section: Clamp malformed policy values so one goal cannot request unbounded motion.
    const auto step = std::clamp(local_scan_step_deg_, 5.0, 90.0);
    constexpr double kMaximumLocalSearchYawDeg = 3.0 * 360.0;
    const auto maximum =
        std::clamp(local_scan_max_yaw_deg_, step, kMaximumLocalSearchYawDeg);
    std::vector<double> headings;
    // Include the terminal heading so the last round closes at the arrival heading.
    for (double heading = 0.0; heading <= maximum; heading += step)
      headings.push_back(heading);
    return headings.empty() ? std::vector<double>{0.0} : headings;
  }

  // Special function: Convert a target's image displacement into a small base-facing yaw.
  double target_facing_yaw(const Detection &detection) const {
    // Section: REP-103 positive yaw turns left, whereas positive image-u lies right.
    if (detection.image_width <= 0) return 0.0;
    const auto normalized_offset = (2.0 * detection.center_u /
                                    static_cast<double>(detection.image_width)) - 1.0;
    const auto half_fov = std::clamp(get_parameter("camera_horizontal_fov_deg").as_double(),
                                     1.0, 179.0) * M_PI / 360.0;
    return -std::atan(normalized_offset * std::tan(half_fov));
  }

  // Special structure: Return target evidence and perception health from one stationary look.
  struct Observation final {
    std::optional<Detection> detection;
    bool saw_frame{false};
  };

  // Special function: Confirm a target from fresh frames while stationary at one viewpoint.
  Observation observe_target(const std::shared_ptr<GoalHandle> &goal_handle,
                             const Action::Goal &goal,
                             const std::set<std::string> &labels, const bool wildcard,
                             const Json &query,
                             const std::chrono::steady_clock::time_point deadline,
                             const std::string &room,
                             const std::string &viewpoint) {
    // Section: Each scan heading starts with fresh detector frames and stable evidence.
    Observation observation;
    std::uint64_t cursor = sequence_;
    unsigned hits = 0;
    std::string last_candidate_key;
    const auto started = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < deadline && active_.load() &&
           !goal_handle->is_canceling()) {
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
      observation.saw_frame = true;
      const auto candidate = select_frame_candidate(frame, labels, wildcard, query);
      if (candidate) {
        const auto key = candidate_key(*candidate);
        hits = key == last_candidate_key ? hits + 1U : 1U;
        last_candidate_key = key;
        observation.detection = candidate;
      } else {
        hits = 0;
        last_candidate_key.clear();
      }
      auto feedback = std::make_shared<Action::Feedback>();
      feedback->phase = observation.detection ? "VERIFYING" : "SEARCHING";
      feedback->current_room = room;
      feedback->current_viewpoint = viewpoint;
      feedback->candidate_confidence = observation.detection ? observation.detection->confidence : 0.0F;
      feedback->elapsed_sec = static_cast<float>(std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count());
      goal_handle->publish_feedback(feedback);
      publish_status(feedback->phase, &goal, feedback->candidate_confidence, false, -1);
      if (hits >= 3U) return observation;
    }
    observation.detection.reset();
    return observation;
  }

  // Special function: Resolve object/category symbols into detector labels and wildcard
  // behavior.
  std::set<std::string> target_labels_for(const std::string &raw_object,
                                          const std::string &raw_category,
                                          bool &wildcard) const {
    // Section: Exact-object aliases take precedence over category membership.
    wildcard = false;
    const auto object = clean_symbol(raw_object);
    if (!object.empty()) {
      if (wildcards_.count(object) != 0U) {
        wildcard = true;
        return {};
      }
      const auto found = object_aliases_.find(object);
      return found == object_aliases_.end() ? std::set<std::string>{}
                                            : std::set<std::string>{found->second};
    }
    const auto category = clean_symbol(raw_category);
    // Section: Generic object/any_object selectors are valid whether transported as
    // object or category.
    if (wildcards_.count(category) != 0U) {
      wildcard = true;
      return {};
    }
    const auto alias = category_aliases_.find(category);
    if (alias == category_aliases_.end()) return {};
    const auto labels = category_labels_.find(alias->second);
    return labels == category_labels_.end() ? std::set<std::string>{} : labels->second;
  }

  // Special function: Resolve one action goal into detector labels using the shared
  // lookup policy.
  std::set<std::string> target_labels(const Action::Goal &goal, bool &wildcard) const {
    return target_labels_for(goal.object_name, goal.category, wildcard);
  }

  // Special function: Normalize optional query JSON while binding it to the action
  // goal's canonical target.
  Json query_from_goal(const Action::Goal &goal) const {
    // Section: Invalid or non-object optional JSON becomes an empty constraint object,
    // never model text.
    Json query = Json::object();
    if (!goal.query_json.empty()) {
      try {
        query = Json::parse(goal.query_json);
      } catch (const Json::exception &) {
        query = Json::object();
      }
    }
    if (!query.is_object()) query = Json::object();
    Json target = query.value("target", Json::object());
    if (!target.is_object()) target = Json::object();
    const auto object = clean_symbol(goal.object_name);
    const auto category = clean_symbol(goal.category);
    if (!object.empty()) target["object_name"] = object;
    if (!category.empty()) target["category"] = category;
    query["target"] = target;
    Json filters = query.value("filters", Json::array());
    if (!filters.is_array()) filters = Json::array();
    // Section: Preserve direct legacy mode payloads while standardizing them as a size
    // constraint.
    auto mode = clean_symbol(query.value("mode", std::string{}));
    if (mode == "biggest") mode = "largest";
    if ((mode == "largest" || mode == "smallest") &&
        std::none_of(filters.begin(), filters.end(), [&mode](const Json &item) {
          return item.is_object() &&
                 item.value("feature", std::string{}) == "size_relation";
        })) {
      filters.push_back(
          Json{{"feature", "size_relation"}, {"operator", "equals"}, {"value", mode}});
    }
    query["filters"] = filters;
    return query;
  }

  // Special function: Read one normalized value from the generic FIND_OBJECT filter
  // list.
  std::string query_filter_value(const Json &query, const std::string &feature) const {
    const auto filters = query.value("filters", Json::array());
    if (!filters.is_array()) return "";
    for (const auto &item : filters) {
      if (!item.is_object()) continue;
      if (clean_symbol(item.value("feature", std::string{})) != feature) continue;
      if (clean_symbol(item.value("operator", std::string{"equals"})) != "equals")
        continue;
      const auto value = clean_symbol(item.value("value", std::string{}));
      if (!value.empty()) return value;
    }
    return "";
  }

  // Special function: Decode optional Qwen semantic attributes without trusting
  // malformed detector data.
  Json detection_attributes(const Detection &detection) const {
    if (detection.attributes_json.empty()) return Json::object();
    try {
      const auto parsed = Json::parse(detection.attributes_json);
      return parsed.is_object() ? parsed : Json::object();
    } catch (const Json::exception &) {
      return Json::object();
    }
  }

  // Special function: Read one semantic property through compatible Qwen schema names.
  std::string detection_attribute(const Detection &detection,
                                  const std::string &feature) const {
    const auto attributes = detection_attributes(detection);
    std::vector<std::string> keys;
    if (feature == "object_color")
      keys = {"color", "object_color"};
    else if (feature == "shape")
      keys = {"shape", "approximate_shape"};
    else if (feature == "orientation")
      keys = {"orientation"};
    for (const auto &key : keys) {
      const auto value = clean_symbol(attributes.value(key, std::string{}));
      if (!value.empty()) return value;
    }
    return "";
  }

  // Special function: Reject candidates that cannot prove requested Qwen
  // characteristics.
  bool matches_attribute_filters(const Detection &detection, const Json &query) const {
    // Section: Size and spatial relations are evaluated after all ordinary attributes
    // are filtered.
    for (const auto &feature : {std::string{"object_color"}, std::string{"shape"},
                                std::string{"orientation"}}) {
      const auto required = query_filter_value(query, feature);
      if (required.empty()) continue;
      if (detection_attribute(detection, feature) != required) return false;
    }
    return true;
  }

  // Special function: Test one supported image-space relation between a target and
  // reference ROI.
  bool relation_matches(const Detection &candidate, const Detection &reference,
                        const std::string &relation) const {
    if (relation == "left_of") return candidate.center_u < reference.center_u;
    if (relation == "right_of") return candidate.center_u > reference.center_u;
    if (relation == "above") return candidate.center_v < reference.center_v;
    if (relation == "below") return candidate.center_v > reference.center_v;
    return relation == "near" || relation == "far_from";
  }

  // Special function: Compute an image-plane center distance for near/far relation
  // ranking.
  double image_distance(const Detection &first, const Detection &second) const {
    const double horizontal = static_cast<double>(first.center_u - second.center_u);
    const double vertical = static_cast<double>(first.center_v - second.center_v);
    return std::sqrt(horizontal * horizontal + vertical * vertical);
  }

  // Special function: Return one finite image ROI area for largest/smallest selection.
  double bbox_area(const Detection &detection) const {
    const double width = std::max(1, detection.x2 - detection.x1);
    const double height = std::max(1, detection.y2 - detection.y1);
    return width * height;
  }

  // Special function: Select a query-compliant target from one detector frame
  // deterministically.
  std::optional<Detection> select_frame_candidate(const std::vector<Detection> &frame,
                                                  const std::set<std::string> &labels,
                                                  const bool wildcard,
                                                  const Json &query) const {
    // Section: First retain only semantic target candidates with sufficiently confident
    // visual evidence.
    std::vector<const Detection *> candidates;
    for (const auto &detection : frame) {
      const auto name = clean_symbol(detection.name);
      const bool semantic_match = wildcard || labels.count(name) != 0U;
      if (semantic_match && detection.confidence >= 0.50F &&
          matches_attribute_filters(detection, query)) {
        candidates.push_back(&detection);
      }
    }
    if (candidates.empty()) return std::nullopt;

    const auto relation = query_filter_value(query, "spatial_relation");
    std::vector<const Detection *> references;
    if (!relation.empty()) {
      bool reference_wildcard = false;
      const auto reference_labels = target_labels_for(
          query_filter_value(query, "reference_object"),
          query_filter_value(query, "reference_category"), reference_wildcard);
      // Section: A relation without a visible configured reference must not select an
      // arbitrary target.
      if (!reference_wildcard && reference_labels.empty()) return std::nullopt;
      for (const auto &detection : frame) {
        const auto name = clean_symbol(detection.name);
        if ((reference_wildcard || reference_labels.count(name) != 0U) &&
            detection.confidence >= 0.50F) {
          references.push_back(&detection);
        }
      }
      if (references.empty()) return std::nullopt;
    }

    const auto size_relation = query_filter_value(query, "size_relation");
    const Detection *best = nullptr;
    double best_relation_score = 0.0;
    for (const auto *candidate : candidates) {
      double relation_score = 0.0;
      if (!relation.empty()) {
        bool relation_satisfied = false;
        double nearest_distance = std::numeric_limits<double>::infinity();
        double farthest_distance = 0.0;
        for (const auto *reference : references) {
          if (candidate == reference ||
              !relation_matches(*candidate, *reference, relation))
            continue;
          relation_satisfied = true;
          const auto distance = image_distance(*candidate, *reference);
          nearest_distance = std::min(nearest_distance, distance);
          farthest_distance = std::max(farthest_distance, distance);
        }
        if (!relation_satisfied) continue;
        relation_score = relation == "far_from" ? farthest_distance : nearest_distance;
      }
      if (best == nullptr) {
        best = candidate;
        best_relation_score = relation_score;
        continue;
      }
      // Section: Near/far ranks relative geometry first; size ranks eligible
      // same-relation candidates.
      if (relation == "near" && relation_score != best_relation_score) {
        if (relation_score < best_relation_score) {
          best = candidate;
          best_relation_score = relation_score;
        }
        continue;
      }
      if (relation == "far_from" && relation_score != best_relation_score) {
        if (relation_score > best_relation_score) {
          best = candidate;
          best_relation_score = relation_score;
        }
        continue;
      }
      const auto candidate_area = bbox_area(*candidate);
      const auto best_area = bbox_area(*best);
      if (size_relation == "smallest" && candidate_area != best_area) {
        if (candidate_area < best_area) best = candidate;
        continue;
      }
      if (size_relation == "largest" && candidate_area != best_area) {
        if (candidate_area > best_area) best = candidate;
        continue;
      }
      if (candidate->confidence > best->confidence) best = candidate;
    }
    return best ? std::optional<Detection>{*best} : std::nullopt;
  }

  // Special function: Build a coarse stable key so three confirmation frames refer to
  // one image target.
  std::string candidate_key(const Detection &detection) const {
    return clean_symbol(detection.name) + "_" +
           std::to_string(static_cast<int>(detection.center_u / 64.0F)) + "_" +
           std::to_string(static_cast<int>(detection.center_v / 64.0F));
  }

  // Special function: Derive a reportable image-grid position from detector-provided
  // image bounds.
  std::string image_position(const Detection &detection) const {
    if (detection.image_width <= 0 || detection.image_height <= 0) return "unknown";
    const double horizontal =
        detection.center_u / static_cast<double>(detection.image_width);
    const double vertical =
        detection.center_v / static_cast<double>(detection.image_height);
    const std::string horizontal_name = horizontal < 1.0 / 3.0   ? "left"
                                        : horizontal > 2.0 / 3.0 ? "right"
                                                                 : "center";
    const std::string vertical_name = vertical < 1.0 / 3.0   ? "upper"
                                      : vertical > 2.0 / 3.0 ? "lower"
                                                             : "middle";
    return vertical_name + "_" + horizontal_name;
  }

  // Special function: Serialize selected visual evidence for speech, history, and
  // downstream manipulation.
  Json selection_payload(const Detection &detection, const Json &query) const {
    // Section: Preserve the original query so a later reporter can explain how the
    // object was selected.
    const auto size_relation = query_filter_value(query, "size_relation");
    const auto spatial_relation = query_filter_value(query, "spatial_relation");
    const auto reason = !size_relation.empty() ? "image_bbox_" + size_relation
                        : !spatial_relation.empty()
                            ? "image_relation_" + spatial_relation
                            : "highest_confidence";
    Json payload{
        {"object_id", candidate_key(detection)},
        {"name", clean_symbol(detection.name)},
        {"confidence", detection.confidence},
        {"bbox", Json::array({detection.x1, detection.y1, detection.x2, detection.y2})},
        {"image_position", image_position(detection)},
        {"attributes", detection_attributes(detection)},
        {"selection_reason", reason},
        {"query", query}};
    payload["furniture_relative_location"] = localize_and_ground(detection);
    if (payload["furniture_relative_location"].value("valid", false)) {
      auto reference = payload["furniture_relative_location"]["reference_name"].get<std::string>();
      std::replace(reference.begin(), reference.end(), '_', ' ');
      payload["furniture_relative_phrase"] = "on the " + reference;
    }
    return payload;
  }

  struct FurniturePrior {
    std::string name;
    double x{}, y{}, yaw{}, size_x{}, size_y{}, top_z{};
  };

  bool load_furniture_map() {
    furniture_priors_.clear();
    object_heights_.clear();
    const auto path = get_parameter("furniture_map_file").as_string();
    if (path.empty()) return true;
    try {
      const auto root = YAML::LoadFile(path);
      for (const auto &entry : root["furniture"]) {
        FurniturePrior prior;
        prior.name = entry["name"].as<std::string>();
        prior.x = entry["x"].as<double>(); prior.y = entry["y"].as<double>();
        prior.yaw = entry["yaw"].as<double>();
        prior.size_x = entry["size_x"].as<double>();
        prior.size_y = entry["size_y"].as<double>();
        prior.top_z = entry["top_z"].as<double>();
        furniture_priors_.push_back(prior);
      }
      if (root["object_heights_m"])
        for (const auto &entry : root["object_heights_m"])
          object_heights_[clean_symbol(entry.first.as<std::string>())] = entry.second.as<double>();
      return true;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Cannot load mapped furniture priors: %s", error.what());
      return false;
    }
  }

  Json localize_and_ground(const Detection &detection) const {
    Json invalid{{"valid", false}, {"target_source", "rgbd_detection"},
                 {"reference_source", "mapped_furniture_prior"}};
    if (!get_parameter("enable_depth").as_bool()) {
      invalid["reason"] = "DEPTH_DISABLED"; return invalid;
    }
    sensor_msgs::msg::Image::SharedPtr depth;
    sensor_msgs::msg::CameraInfo::SharedPtr info;
    { std::lock_guard<std::mutex> lock(depth_mutex_); depth = depth_image_; info = camera_info_; }
    if (!depth || !info) { invalid["reason"] = "NO_REGISTERED_DEPTH"; return invalid; }
    const rclcpp::Time detection_time(detection.stamp);
    const rclcpp::Time depth_time(depth->header.stamp);
    if (std::abs((depth_time - detection_time).seconds()) >
        get_parameter("depth_max_age_sec").as_double()) {
      invalid["reason"] = "STALE_DEPTH"; return invalid;
    }
    if (info->k[0] <= 0.0 || info->k[4] <= 0.0) {
      invalid["reason"] = "INVALID_CAMERA_INFO"; return invalid;
    }
    cv_bridge::CvImageConstPtr cv_depth;
    try { cv_depth = cv_bridge::toCvShare(depth); }
    catch (const cv_bridge::Exception &) { invalid["reason"] = "UNSUPPORTED_DEPTH"; return invalid; }
    const int u0 = std::max(0, static_cast<int>(detection.center_u) - 10);
    const int v0 = std::max(0, static_cast<int>(detection.center_v) - 10);
    const int u1 = std::min(static_cast<int>(depth->width), u0 + 20);
    const int v1 = std::min(static_cast<int>(depth->height), v0 + 20);
    std::vector<float> ranges;
    for (int v = v0; v < v1; ++v) for (int u = u0; u < u1; ++u) {
      float value = std::numeric_limits<float>::quiet_NaN();
      if (depth->encoding == "32FC1") value = cv_depth->image.at<float>(v, u);
      else if (depth->encoding == "16UC1") value = cv_depth->image.at<std::uint16_t>(v, u) * 0.001F;
      if (std::isfinite(value) && value > 0.15F && value < 6.0F) ranges.push_back(value);
    }
    const auto sample_count = std::max(1, (u1-u0)*(v1-v0));
    if (ranges.size() / static_cast<double>(sample_count) <
        get_parameter("depth_min_valid_ratio").as_double()) {
      invalid["reason"] = "INSUFFICIENT_VALID_DEPTH"; return invalid;
    }
    std::nth_element(ranges.begin(), ranges.begin()+ranges.size()/2, ranges.end());
    const double z = ranges[ranges.size()/2];
    geometry_msgs::msg::PointStamped optical, mapped;
    optical.header = depth->header;
    optical.point.z = z;
    optical.point.x = (detection.center_u - info->k[2]) * z / info->k[0];
    optical.point.y = (detection.center_v - info->k[5]) * z / info->k[4];
    try { mapped = tf_buffer_->transform(optical, "map", tf2::durationFromSec(0.3)); }
    catch (const tf2::TransformException &error) {
      invalid["reason"] = "TF_FAILED"; invalid["detail"] = error.what(); return invalid;
    }
    const auto height = object_heights_.find(clean_symbol(detection.name));
    if (height == object_heights_.end()) {
      invalid["reason"] = "UNKNOWN_OBJECT_HEIGHT"; return invalid;
    }
    const double bottom_z = mapped.point.z - height->second / 2.0;
    for (const auto &prior : furniture_priors_) {
      const double dx = mapped.point.x-prior.x, dy = mapped.point.y-prior.y;
      const double local_x = std::cos(prior.yaw)*dx + std::sin(prior.yaw)*dy;
      const double local_y = -std::sin(prior.yaw)*dx + std::cos(prior.yaw)*dy;
      const bool overlap = std::abs(local_x) <= prior.size_x/2.0 &&
                           std::abs(local_y) <= prior.size_y/2.0;
      const bool supported = std::abs(bottom_z-prior.top_z) <=
          get_parameter("support_height_tolerance_m").as_double();
      if (overlap && supported) return Json{{"valid", true}, {"relation", "on"},
          {"reference_name", prior.name}, {"frame_id", "map"},
          {"target_source", "rgbd_detection"},
          {"reference_source", "mapped_furniture_prior"},
          {"target_point", {{"x", mapped.point.x}, {"y", mapped.point.y}, {"z", mapped.point.z}}},
          {"estimated_bottom_z", bottom_z}, {"support_top_z", prior.top_z},
          {"depth_m", z}, {"depth_stamp_sec", depth_time.seconds()}};
    }
    invalid["reason"] = "NO_SUPPORTING_FURNITURE";
    invalid["target_point"] = {{"x", mapped.point.x}, {"y", mapped.point.y}, {"z", mapped.point.z}};
    return invalid;
  }

  // Special function: Send a sanitized goal query to the lifecycle-owned Qwen detector.
  void publish_detector_query(const Json &query) {
    if (!detector_query_publisher_) return;
    std_msgs::msg::String message;
    message.data = query.dump();
    detector_query_publisher_->publish(message);
  }

  // Special function: Retain one complete detector frame, including an empty frame.
  void detections_callback(const DetectionArray::SharedPtr message) {
    if (!active_.load()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    detections_ = message->detections;
    ++sequence_;
    ++detection_frame_count_;
    last_detection_time_ = std::chrono::steady_clock::now();
    condition_.notify_all();
  }

  // Special function: Track the detector's transient-local live-inference readiness.
  void ready_callback(const std_msgs::msg::Bool::SharedPtr message) {
    detector_ready_.store(message->data);
    // Section: Retain readiness delivery evidence separately from its latest value.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++ready_message_count_;
      last_ready_time_ = std::chrono::steady_clock::now();
    }
    condition_.notify_all();
  }

  // Special function: Build actionable detector-wiring evidence for terminal reports.
  Json perception_diagnostics(const std::uint64_t frames_at_goal_start,
                              const bool observation_attempted) {
    // Section: Snapshot callback-owned timestamps under the same lock used for frames.
    std::optional<std::chrono::steady_clock::time_point> last_detection;
    std::optional<std::chrono::steady_clock::time_point> last_ready;
    std::uint64_t ready_messages = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_detection = last_detection_time_;
      last_ready = last_ready_time_;
      ready_messages = ready_message_count_;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto age_seconds = [&now](const auto &time) -> Json {
      return time ? Json(std::chrono::duration<double>(now - *time).count()) : Json(nullptr);
    };
    // Section: Include endpoint counts to distinguish missing publishers from stale streams.
    return Json{{"detector_ready", detector_ready_.load()},
                {"ready_messages_received", ready_messages},
                {"detection_frames_received", detection_frame_count_.load()},
                {"detection_frames_during_goal",
                 detection_frame_count_.load() - frames_at_goal_start},
                {"last_ready_age_sec", age_seconds(last_ready)},
                {"last_detection_age_sec", age_seconds(last_detection)},
                {"observation_attempted", observation_attempted},
                {"ready_topic", get_parameter("detector_ready_topic").as_string()},
                {"detections_topic", get_parameter("detections_topic").as_string()},
                {"ready_publishers", ready_subscription_ ? ready_subscription_->get_publisher_count() : 0U},
                {"detection_publishers", detection_subscription_ ? detection_subscription_->get_publisher_count() : 0U}};
  }

  // Special function: Reject malformed, inactive, or concurrent object missions.
  rclcpp_action::GoalResponse admit(const rclcpp_action::GoalUUID &,
                                    const std::shared_ptr<const Action::Goal> goal) {
    // Section: Resolve the implicit default scope before validating goal routing.
    const auto scope = goal->search_scope.empty()
                           ? (goal->location.empty() ? "GLOBAL" : "STRICT_LOCATION")
                           : goal->search_scope;
    // Section: Reject goals that cannot identify any target object or category.
    if (goal->object_name.empty() && goal->category.empty()) {
      RCLCPP_WARN(get_logger(), "Rejected FindObject goal: object_name and category are empty.");
      publish_status("REJECTED", goal.get(), 0.0, true, -1, "", "EMPTY_TARGET");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Section: LOCAL searches rotate at the current pose; strict searches need a room.
    if (scope != "LOCAL" && scope != "GLOBAL" &&
        !(scope == "STRICT_LOCATION" && !goal->location.empty())) {
      RCLCPP_WARN(get_logger(), "Rejected FindObject goal: invalid scope or missing location.");
      publish_status("REJECTED", goal.get(), 0.0, true, -1, "", "INVALID_SCOPE");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Section: Accept action work only while the lifecycle server is ACTIVE.
    if (!active_.load()) {
      RCLCPP_WARN(get_logger(), "Rejected FindObject goal: lifecycle server is inactive.");
      publish_status("REJECTED", goal.get(), 0.0, true, -1, "", "INACTIVE");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Section: Atomically acquire exclusive ownership for exactly one mission worker.
    bool expected_busy = false;
    if (!busy_.compare_exchange_strong(expected_busy, true)) {
      RCLCPP_WARN(get_logger(), "Rejected FindObject goal: another mission is running.");
      publish_status("REJECTED", goal.get(), 0.0, true, -1, "", "BUSY");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Section: The accepted worker now owns the guard that will release this lock.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Special function: Move the bounded perception mission off executor callback
  // threads.
  void execute_async(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  // Special function: Confirm the selected semantic target across a sliding frame
  // window.
  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    // Section: Capture the immutable goal before entering the exception-safe worker body.
    const auto goal = goal_handle->get_goal();
    // Section: Ensure every normal or exceptional worker exit releases busy_.
    const BusyResetGuard busy_reset(this);
    try {
      bool wildcard = false;
      const auto labels = target_labels(*goal, wildcard);
      // Section: The action goal is the sole source of the active Qwen target/query for
      // this mission.
      const auto query = query_from_goal(*goal);
      publish_detector_query(query);
      const auto timeout =
          goal->timeout_sec > 0.0F ? goal->timeout_sec : default_timeout_sec_;
      // Section: Measure fresh detector traffic only after this action goal begins.
      const auto frames_at_goal_start = detection_frame_count_.load();
      const auto started = std::chrono::steady_clock::now();
      const auto deadline = started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                          std::chrono::duration<double>(timeout));
      publish_status("GOAL_ACCEPTED", goal.get(), 0.0, false, -1);
      std::optional<Detection> best;
      bool saw_frame = false;
      bool observation_attempted = false;
      bool facing_succeeded = true;
      unsigned navigation_failures = 0;
      std::string found_viewpoint;
      std::string found_room;
      std::vector<std::string> viewpoints_searched;
      const bool navigation_enabled = get_parameter("enable_navigation").as_bool();
      const bool local_scan_only = goal->search_scope == "LOCAL";
      // Section: LOCAL retains Nav2 Spin but deliberately skips named-waypoint travel.
      const bool waypoint_navigation_enabled = navigation_enabled && !local_scan_only;
      auto viewpoints = waypoint_navigation_enabled ? search_viewpoints(*goal)
                                                    : std::vector<std::string>{"current_pose"};
      if (waypoint_navigation_enabled && !goal->preferred_viewpoint.empty()) {
        const auto preferred = clean_symbol(goal->preferred_viewpoint);
        viewpoints.erase(std::remove(viewpoints.begin(), viewpoints.end(), preferred),
                        viewpoints.end());
        viewpoints.insert(viewpoints.begin(), preferred);
      }
      for (const auto &viewpoint : viewpoints) {
        if (std::chrono::steady_clock::now() >= deadline || !active_.load() ||
            goal_handle->is_canceling()) break;
        const auto room = waypoint_navigation_enabled ? clean_symbol(goal->location) : "";
        if (waypoint_navigation_enabled) {
          publish_status("NAVIGATING", goal.get(), 0.0, false, -1);
          if (!navigate_to_viewpoint(viewpoint)) {
            ++navigation_failures;
            continue;
          }
          viewpoints_searched.push_back(viewpoint);
        } else if (local_scan_only) {
          // Section: Report the current-pose scan as a searched viewpoint.
          viewpoints_searched.push_back(viewpoint);
        }
        double last_angle_deg = 0.0;
        // Section: LOCAL performs a complete incremental sweep; other scopes retain
        // the configured short waypoint scan.
        const auto headings = local_scan_only ? local_scan_headings() : scan_angles_deg_;
        for (const auto angle_deg : headings) {
          if (std::chrono::steady_clock::now() >= deadline || !active_.load() ||
              goal_handle->is_canceling()) break;
          // Section: Nav2 Spin consumes a relative yaw, while policy headings are
          // offsets from the waypoint arrival orientation.
          if (navigation_enabled && !spin_by((angle_deg - last_angle_deg) * M_PI / 180.0)) {
            ++navigation_failures;
            break;
          }
          last_angle_deg = angle_deg;
          observation_attempted = true;
          const auto observation_deadline = navigation_enabled
              ? std::min(deadline, std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(confirmation_timeout_sec_)))
              : deadline;
          const auto observation = observe_target(goal_handle, *goal, labels, wildcard,
                                                  query, observation_deadline, room,
                                                  viewpoint);
          saw_frame = saw_frame || observation.saw_frame;
          if (observation.detection) {
            best = observation.detection;
            found_viewpoint = viewpoint;
            found_room = room;
            break;
          }
        }
        if (best) break;
      }
      // Section: After stable verification, center the target before returning success.
      if (best && local_scan_only && navigation_enabled) {
        const auto correction_yaw = target_facing_yaw(*best);
        if (!spin_by(correction_yaw)) {
          ++navigation_failures;
          facing_succeeded = false;
        }
      }
      auto result = std::make_shared<Action::Result>();
      if (goal_handle->is_canceling() || !active_.load()) {
      result->result_code = Action::Result::CANCELED;
      result->message = "FindObject mission canceled.";
      publish_status("CANCELED", goal.get(), best ? best->confidence : 0.0, true,
                     result->result_code);
      goal_handle->canceled(result);
      } else if (waypoint_navigation_enabled && !viewpoints.empty() && !observation_attempted) {
      const auto diagnostics = perception_diagnostics(frames_at_goal_start, false);
      result->result_code = Action::Result::NAVIGATION_FAILED;
      result->navigation_failures = navigation_failures;
      result->viewpoints_searched = viewpoints_searched;
      result->message = "NAVIGATION_PREVENTED_OBSERVATION: no waypoint scan phase was reached.";
      publish_status("NAVIGATION_FAILED", goal.get(), 0.0, true, result->result_code, "",
                     "NAVIGATION_PREVENTED_OBSERVATION", diagnostics);
      goal_handle->abort(result);
      } else if (!detector_ready_.load() || !saw_frame) {
      const auto diagnostics = perception_diagnostics(frames_at_goal_start,
                                                      observation_attempted);
      result->result_code = Action::Result::PERCEPTION_FAILED;
      // Section: Report the one failed contract rather than merging readiness and frames.
      const std::string reason = !detector_ready_.load()
          ? (diagnostics["ready_messages_received"].get<std::uint64_t>() == 0U
                 ? "DETECTOR_NOT_READY_NO_READY_MESSAGE"
                 : "DETECTOR_REPORTED_NOT_READY")
          : "NO_FRESH_DETECTION_FRAMES";
      result->message = reason + ": ready_topic=" +
          diagnostics["ready_topic"].get<std::string>() + ", detections_topic=" +
          diagnostics["detections_topic"].get<std::string>() + ", detection_frames_during_goal=" +
          std::to_string(diagnostics["detection_frames_during_goal"].get<std::uint64_t>()) + ".";
      result->navigation_failures = navigation_failures;
      result->viewpoints_searched = viewpoints_searched;
      publish_status("PERCEPTION_FAILED", goal.get(), 0.0, true, result->result_code,
                     "", reason, diagnostics);
      goal_handle->abort(result);
      } else if (!best) {
      result->result_code = Action::Result::NOT_FOUND;
      result->message = "Target was not confirmed before the search deadline.";
      result->navigation_failures = navigation_failures;
      result->viewpoints_searched = viewpoints_searched;
      publish_status("NOT_FOUND", goal.get(), best ? best->confidence : 0.0, true,
                     result->result_code);
      goal_handle->succeed(result);
      } else {
      result->found = true;
      result->approached = false;
      result->result_code = (goal->approach_target || !facing_succeeded)
          ? Action::Result::FOUND_UNREACHABLE : Action::Result::SUCCESS;
      result->object_id = clean_symbol(best->name) + "_" +
                          std::to_string(static_cast<int>(best->center_u)) + "_" +
                          std::to_string(static_cast<int>(best->center_v));
      result->found_room = found_room;
      result->found_viewpoint = found_viewpoint;
      result->viewpoints_searched = viewpoints_searched;
      result->navigation_failures = navigation_failures;
      const auto selection = selection_payload(*best, query);
      result->selection_json = selection.dump();
      const auto grounding = selection.value("furniture_relative_location", Json::object());
      if (grounding.value("valid", false) && grounding.contains("target_point")) {
        const auto &point = grounding.at("target_point");
        result->object_pose.header.frame_id = "map";
        result->object_pose.header.stamp = now();
        result->object_pose.pose.position.x = point.value("x", 0.0);
        result->object_pose.pose.position.y = point.value("y", 0.0);
        result->object_pose.pose.position.z = point.value("z", 0.0);
        result->object_pose.pose.orientation.w = 1.0;
      }
      result->message = !facing_succeeded
                            ? "Target verified, but the final target-facing spin failed."
                            : goal->approach_target
                            ? "Target verified, but approach geometry is not available "
                              "from RGB detections alone."
                            : local_scan_only && navigation_enabled
                                ? "Target verified and the robot is facing the target."
                                : "Target verified from the live object detector.";
      publish_status((goal->approach_target || !facing_succeeded) ? "FOUND_UNREACHABLE" : "SUCCESS",
                     goal.get(), best->confidence, true, result->result_code,
                     result->object_id);
      goal_handle->succeed(result);
      }
    } catch (const std::exception &error) {
      // Section: Convert unexpected worker failures into a terminal action result.
      RCLCPP_ERROR(get_logger(), "FindObject worker failed: %s", error.what());
      auto result = std::make_shared<Action::Result>();
      result->result_code = Action::Result::PERCEPTION_FAILED;
      result->message = std::string("FindObject worker failed: ") + error.what();
      publish_status("INTERNAL_ERROR", goal.get(), 0.0, true, result->result_code, "",
                     error.what());
      goal_handle->abort(result);
    } catch (...) {
      // Section: Preserve server availability for non-standard worker failures.
      RCLCPP_ERROR(get_logger(), "FindObject worker failed with an unknown exception.");
      auto result = std::make_shared<Action::Result>();
      result->result_code = Action::Result::PERCEPTION_FAILED;
      result->message = "FindObject worker failed with an unknown exception.";
      publish_status("INTERNAL_ERROR", goal.get(), 0.0, true, result->result_code, "",
                     "UNKNOWN_EXCEPTION");
      goal_handle->abort(result);
    }
  }

  // Special function: Publish the transient target-state JSON contract for observers.
  void publish_status(const std::string &phase, const Action::Goal *goal,
                      const double confidence, const bool terminal,
                      const int result_code, const std::string &object_id = "",
                      const std::string &reason = "",
                      const Json &diagnostics = Json::object()) {
    Json payload{{"phase", phase},
                 {"target_name", goal ? clean_symbol(goal->object_name) : ""},
                 {"category", goal ? clean_symbol(goal->category) : ""},
                 {"location", goal ? clean_symbol(goal->location) : ""},
                 {"detected", confidence > 0.0},
                 {"verified", terminal && confidence > 0.0},
                 {"found", terminal && (result_code == 0 || result_code == 2 ||
                                        result_code == 3)},
                 {"confidence", confidence},
                 {"object_id", object_id},
                 {"found_room", terminal && goal ? clean_symbol(goal->location) : ""},
                 {"found_viewpoint", ""},
                 {"result_code", result_code},
                 {"terminal", terminal},
                 {"busy", busy_.load()},
                 {"reason", reason},
                 {"diagnostics", diagnostics}};
    std_msgs::msg::String message;
    message.data = payload.dump();
    status_publisher_->publish(message);
  }

  // Special function: Reset every lifecycle-owned endpoint and volatile sample.
  void release_resources() {
    action_server_.reset();
    detection_subscription_.reset();
    ready_subscription_.reset();
    depth_subscription_.reset();
    camera_info_subscription_.reset();
    detector_query_publisher_.reset();
    named_navigation_client_.reset();
    spin_client_.reset();
    std::lock_guard<std::mutex> lock(mutex_);
    detections_.clear();
    sequence_ = 0;
    detection_frame_count_.store(0);
    ready_message_count_ = 0;
    last_detection_time_.reset();
    last_ready_time_.reset();
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
  // Section: Search policy state separates strict room lookup from global route order.
  std::map<std::string, std::vector<std::string>> room_viewpoints_;
  std::vector<std::string> global_viewpoints_;
  std::vector<double> scan_angles_deg_;
  double default_timeout_sec_{300.0};
  double navigation_server_timeout_sec_{5.0};
  double navigation_result_timeout_sec_{120.0};
  double scan_action_timeout_sec_{20.0};
  double confirmation_timeout_sec_{3.0};
  double local_scan_step_deg_{15.0};
  double local_scan_max_yaw_deg_{1080.0};
  std::mutex mutex_;
  std::condition_variable condition_;
  std::uint64_t sequence_{0};
  std::atomic_uint64_t detection_frame_count_{0};
  std::uint64_t ready_message_count_{0};
  std::optional<std::chrono::steady_clock::time_point> last_detection_time_;
  std::optional<std::chrono::steady_clock::time_point> last_ready_time_;
  std::vector<Detection> detections_;
  mutable std::mutex depth_mutex_;
  sensor_msgs::msg::Image::SharedPtr depth_image_;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  std::vector<FurniturePrior> furniture_priors_;
  std::map<std::string, double> object_heights_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detector_query_publisher_;
  rclcpp::Subscription<DetectionArray>::SharedPtr detection_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp_action::Client<NavigateNamedPose>::SharedPtr named_navigation_client_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
};

}  // namespace robot_yasmin_tasks

// Section: Process startup and lifecycle object-search action-server execution.
int main(int argc, char **argv) {
  // Section: Four executor threads keep perception callbacks responsive during action
  // work.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_yasmin_tasks::FindObjectServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
