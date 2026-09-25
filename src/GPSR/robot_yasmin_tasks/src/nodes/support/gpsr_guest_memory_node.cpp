#include <algorithm>
#include <cctype>
#include <chrono>
#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <robot_ai_interfaces/msg/found_entity_record.hpp>
#include <robot_ai_interfaces/msg/gpsr_guest_record.hpp>
#include <robot_ai_interfaces/msg/person_profile.hpp>
#include <robot_ai_interfaces/srv/get_gpsr_guest.hpp>
#include <robot_ai_interfaces/srv/list_gpsr_guests.hpp>
#include <robot_ai_interfaces/srv/lookup_found_entity.hpp>
#include <robot_ai_interfaces/srv/save_found_entity.hpp>
#include <robot_ai_interfaces/srv/save_gpsr_guest.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <string>

namespace robot_yasmin_tasks {

using Json = nlohmann::json;
using FoundRecord = robot_ai_interfaces::msg::FoundEntityRecord;
using GuestRecord = robot_ai_interfaces::msg::GpsrGuestRecord;

// Special function: Normalize lookup keys exactly once at the memory boundary.
std::string normalize_key(std::string value, const std::size_t limit) {
  if (value.size() > limit) value.resize(limit);
  value.erase(value.begin(),
              std::find_if(value.begin(), value.end(),
                           [](unsigned char c) { return !std::isspace(c); }));
  value.erase(std::find_if(value.rbegin(), value.rend(),
                           [](unsigned char c) { return !std::isspace(c); })
                  .base(),
              value.end());
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

// Special function: Parse only JSON objects so malformed input cannot corrupt memory.
Json parse_object(const std::string &raw) {
  try {
    auto parsed = Json::parse(raw.empty() ? "{}" : raw);
    return parsed.is_object() ? parsed : Json::object();
  } catch (const Json::exception &) {
    return Json::object();
  }
}

// Special function: Decide whether an incoming semantic value carries useful evidence.
bool meaningful(const Json &value) {
  if (value.is_null()) return false;
  if (value.is_string()) {
    const auto normalized = normalize_key(value.get<std::string>(), 4096);
    return !normalized.empty() && normalized != "unknown";
  }
  if ((value.is_array() || value.is_object()) && value.empty()) return false;
  return true;
}

// Special function: Recursively retain known evidence while merging useful updates.
Json merge_nonempty(Json existing, const Json &incoming) {
  if (existing.is_object() && incoming.is_object()) {
    for (auto iterator = incoming.begin(); iterator != incoming.end(); ++iterator) {
      existing[iterator.key()] =
          merge_nonempty(existing.value(iterator.key(), Json()), iterator.value());
    }
    return existing;
  }
  return meaningful(incoming) ? incoming : existing;
}

// Special function: Compact and bound retained JSON evidence.
std::string compact_object(const Json &value) {
  const auto encoded = (value.is_object() ? value : Json::object()).dump();
  return encoded.size() <= 4096U ? encoded : "{}";
}

// Special function: Generate one process-session identifier without persistent storage.
std::string make_session_id() {
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  std::ostringstream stream;
  stream << std::hex << ticks;
  return stream.str();
}

class GpsrGuestMemoryNode final : public rclcpp::Node {
 public:
  // Special function: Expose bounded GPSR memory through the original service names.
  GpsrGuestMemoryNode()
      : rclcpp::Node("gpsr_guest_memory_node"), session_id_(make_session_id()) {
    max_find_records_ = static_cast<std::size_t>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("max_find_records", 32)));
    max_guest_records_ = static_cast<std::size_t>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("max_guest_records", 32)));
    const auto pose_topic =
        declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
    pose_max_age_sec_ =
        std::max(0.0, declare_parameter<double>("amcl_pose_max_age_sec", 1.0));

    // Section: Keep all externally visible endpoint names identical to the Python node.
    save_service_ = create_service<robot_ai_interfaces::srv::SaveGpsrGuest>(
        "/gpsr/guest_memory/save",
        std::bind(&GpsrGuestMemoryNode::save_guest, this, std::placeholders::_1,
                  std::placeholders::_2));
    get_service_ = create_service<robot_ai_interfaces::srv::GetGpsrGuest>(
        "/gpsr/guest_memory/get",
        std::bind(&GpsrGuestMemoryNode::get_guest, this, std::placeholders::_1,
                  std::placeholders::_2));
    list_service_ = create_service<robot_ai_interfaces::srv::ListGpsrGuests>(
        "/gpsr/guest_memory/list",
        std::bind(&GpsrGuestMemoryNode::list_guests, this, std::placeholders::_1,
                  std::placeholders::_2));
    clear_service_ = create_service<std_srvs::srv::Trigger>(
        "/gpsr/guest_memory/clear",
        std::bind(&GpsrGuestMemoryNode::clear_memory, this, std::placeholders::_1,
                  std::placeholders::_2));
    find_save_service_ = create_service<robot_ai_interfaces::srv::SaveFoundEntity>(
        "/gpsr/find_history/save",
        std::bind(&GpsrGuestMemoryNode::save_found, this, std::placeholders::_1,
                  std::placeholders::_2));
    find_lookup_service_ = create_service<robot_ai_interfaces::srv::LookupFoundEntity>(
        "/gpsr/find_history/lookup",
        std::bind(&GpsrGuestMemoryNode::lookup_found, this, std::placeholders::_1,
                  std::placeholders::_2));
    find_clear_service_ = create_service<std_srvs::srv::Trigger>(
        "/gpsr/find_history/clear",
        std::bind(&GpsrGuestMemoryNode::clear_memory, this, std::placeholders::_1,
                  std::placeholders::_2));
    pose_subscription_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic, rclcpp::SensorDataQoS(),
            std::bind(&GpsrGuestMemoryNode::cache_pose, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "GPSR guest memory ready for session %s.",
                session_id_.c_str());
  }

 private:
  // Special function: Resolve the same stable profile-or-session identity as the Python
  // store.
  std::string resolve_memory_id(const GuestRecord &incoming) const {
    if (!normalize_key(incoming.memory_id, 4096).empty()) return incoming.memory_id;
    if (!normalize_key(incoming.profile.profile_id, 4096).empty()) {
      return "profile:" + normalize_key(incoming.profile.profile_id, 4096);
    }
    const auto person_ref = parse_object(incoming.person_ref_json);
    const auto profile_id = person_ref.value("profile_id", std::string());
    if (!normalize_key(profile_id, 4096).empty())
      return "profile:" + normalize_key(profile_id, 4096);
    const auto tracking_session =
        !incoming.tracking_session_id.empty()
            ? incoming.tracking_session_id
            : person_ref.value("tracking_session_id", session_id_);
    const auto track_id =
        incoming.track_id != 0 ? incoming.track_id : person_ref.value("track_id", 0);
    return "anonymous:" + tracking_session + ":" + std::to_string(track_id);
  }

  // Special function: Merge confidence-bearing appearance fields without weakening
  // evidence.
  static void merge_profile(robot_ai_interfaces::msg::PersonProfile &stored,
                            const robot_ai_interfaces::msg::PersonProfile &incoming) {
    auto update_text = [](std::string &target, const std::string &source) {
      if (!normalize_key(source, 4096).empty() &&
          normalize_key(source, 4096) != "unknown")
        target = source;
    };
    update_text(stored.profile_id, incoming.profile_id);
    update_text(stored.role, incoming.role);
    update_text(stored.name, incoming.name);
    update_text(stored.gender, incoming.gender);
    update_text(stored.favorite_drink, incoming.favorite_drink);
    update_text(stored.hometown, incoming.hometown);
    update_text(stored.seat_position, incoming.seat_position);
    update_text(stored.clothes_color, incoming.clothes_color);
    if (incoming.age > 0) stored.age = incoming.age;
    if (!incoming.face_embedding.empty()) {
      stored.face_embedding.assign(
          incoming.face_embedding.begin(),
          incoming.face_embedding.begin() +
              std::min<std::size_t>(1024, incoming.face_embedding.size()));
    }
    stored.is_confirmed = stored.is_confirmed || incoming.is_confirmed;
    stored.confidence_score =
        std::max(stored.confidence_score, incoming.confidence_score);
    auto merge_state = [](std::uint8_t &state, float &confidence,
                          const std::uint8_t new_state, const float new_confidence) {
      if (new_state != 0U && (state == 0U || new_confidence >= confidence)) {
        state = new_state;
        confidence = new_confidence;
      }
    };
    merge_state(stored.appearance.glasses, stored.appearance.glasses_confidence,
                incoming.appearance.glasses, incoming.appearance.glasses_confidence);
    merge_state(stored.appearance.hat, stored.appearance.hat_confidence,
                incoming.appearance.hat, incoming.appearance.hat_confidence);
    merge_state(stored.appearance.face_mask, stored.appearance.face_mask_confidence,
                incoming.appearance.face_mask,
                incoming.appearance.face_mask_confidence);
    if (!incoming.appearance.shirt_color.empty() &&
        (stored.appearance.shirt_color.empty() ||
         incoming.appearance.shirt_color_confidence >=
             stored.appearance.shirt_color_confidence)) {
      stored.appearance.shirt_color = incoming.appearance.shirt_color;
      stored.appearance.shirt_color_confidence =
          incoming.appearance.shirt_color_confidence;
    }
  }

  // Special function: Save or update one guest and return the authoritative revision.
  void save_guest(
      const robot_ai_interfaces::srv::SaveGpsrGuest::Request::SharedPtr request,
      robot_ai_interfaces::srv::SaveGpsrGuest::Response::SharedPtr response) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto memory_id = resolve_memory_id(request->record);
    auto &stored = guest_records_[memory_id];
    const bool created = stored.memory_id.empty();
    if (created) stored.created_at = now();
    stored.memory_id = memory_id;
    if (!request->record.command_id.empty())
      stored.command_id = request->record.command_id;
    if (!request->record.source_step_id.empty())
      stored.source_step_id = request->record.source_step_id;
    if (!request->record.tracking_session_id.empty())
      stored.tracking_session_id = request->record.tracking_session_id;
    if (stored.tracking_session_id.empty()) stored.tracking_session_id = session_id_;
    if (request->record.track_id != 0 || stored.track_id == 0)
      stored.track_id = request->record.track_id;
    stored.person_ref_json =
        compact_object(merge_nonempty(parse_object(stored.person_ref_json),
                                      parse_object(request->record.person_ref_json)));
    stored.guest_info_json =
        compact_object(merge_nonempty(parse_object(stored.guest_info_json),
                                      parse_object(request->record.guest_info_json)));
    merge_profile(stored.profile, request->record.profile);
    stored.updated_at = now();
    ++stored.revision;
    active_memory_id_ = memory_id;
    touch_guest(memory_id);
    response->success = true;
    response->record = stored;
    response->message = "Stored GPSR guest " + memory_id + ".";
  }

  // Special function: Select a requested or active guest without mutating memory.
  void get_guest(
      const robot_ai_interfaces::srv::GetGpsrGuest::Request::SharedPtr request,
      robot_ai_interfaces::srv::GetGpsrGuest::Response::SharedPtr response) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto selected =
        request->memory_id.empty() ? active_memory_id_ : request->memory_id;
    const auto iterator = guest_records_.find(selected);
    response->success = iterator != guest_records_.end();
    if (response->success) {
      response->record = iterator->second;
      response->message = "Retrieved GPSR guest " + selected + ".";
    } else {
      response->message =
          "GPSR guest '" +
          (request->memory_id.empty() ? std::string("active") : request->memory_id) +
          "' was not found.";
    }
  }

  // Special function: Return guest records in deterministic insertion order.
  void list_guests(
      const robot_ai_interfaces::srv::ListGpsrGuests::Request::SharedPtr,
      robot_ai_interfaces::srv::ListGpsrGuests::Response::SharedPtr response) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &id : guest_order_) {
      const auto iterator = guest_records_.find(id);
      if (iterator != guest_records_.end())
        response->records.push_back(iterator->second);
    }
    response->success = true;
    response->active_memory_id = active_memory_id_;
    response->message =
        "Returned " + std::to_string(response->records.size()) + " GPSR guest records.";
  }

  // Special function: Upsert one normalized and bounded find-history record.
  void save_found(
      const robot_ai_interfaces::srv::SaveFoundEntity::Request::SharedPtr request,
      robot_ai_interfaces::srv::SaveFoundEntity::Response::SharedPtr response) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto entity_type = normalize_key(request->record.entity_type, 32);
    const auto target_key = normalize_key(request->record.target_key, 256);
    if (entity_type.empty() || target_key.empty()) {
      response->success = false;
      response->message =
          "Find-history save failed: entity_type and target_key are required";
      return;
    }
    const auto key = entity_type + ":" + target_key;
    auto &stored = find_records_[key];
    const bool created = stored.entity_type.empty();
    if (created) stored.created_at = now();
    stored.entity_type = entity_type;
    stored.target_key = target_key;
    stored.entity_id = request->record.entity_id.empty()
                           ? (stored.entity_id.empty() ? target_key : stored.entity_id)
                           : request->record.entity_id;
    if (!request->record.semantic_location.empty())
      stored.semantic_location = request->record.semantic_location.substr(0, 128);
    if (!request->record.viewpoint.empty())
      stored.viewpoint = request->record.viewpoint.substr(0, 128);
    stored.metadata_json = compact_object(parse_object(
        request->record.metadata_json.empty() ? stored.metadata_json
                                              : request->record.metadata_json));
    if (!request->record.entity_pose.header.frame_id.empty())
      stored.entity_pose = request->record.entity_pose;
    if (!request->record.observation_pose.header.frame_id.empty()) {
      stored.observation_pose = request->record.observation_pose;
    } else if (latest_pose_ &&
               (std::chrono::steady_clock::now() - latest_pose_received_) <=
                   std::chrono::duration<double>(pose_max_age_sec_)) {
      stored.observation_pose = *latest_pose_;
    }
    stored.updated_at = now();
    ++stored.revision;
    touch_found(key);
    response->success = true;
    response->record = stored;
    response->message = "Stored find history " + target_key + ".";
  }

  // Special function: Look up an exact normalized entity observation.
  void lookup_found(
      const robot_ai_interfaces::srv::LookupFoundEntity::Request::SharedPtr request,
      robot_ai_interfaces::srv::LookupFoundEntity::Response::SharedPtr response) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto key = normalize_key(request->entity_type, 32) + ":" +
                     normalize_key(request->target_key, 256);
    const auto iterator = find_records_.find(key);
    response->found = iterator != find_records_.end();
    if (response->found) {
      response->record = iterator->second;
      response->message = "Found remembered " + iterator->second.target_key + ".";
    } else {
      response->message = "No matching find-history record.";
    }
  }

  // Special function: Clear all session values while retaining the process session
  // identity.
  void clear_memory(const std_srvs::srv::Trigger::Request::SharedPtr,
                    std_srvs::srv::Trigger::Response::SharedPtr response) {
    std::lock_guard<std::mutex> lock(mutex_);
    guest_records_.clear();
    guest_order_.clear();
    find_records_.clear();
    find_order_.clear();
    active_memory_id_.clear();
    latest_pose_.reset();
    response->success = true;
    response->message = "GPSR guest and find-history memory cleared.";
  }

  // Special function: Cache one map observation pose for a later person-find save.
  void cache_pose(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
    std::lock_guard<std::mutex> lock(mutex_);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = message->header;
    pose.pose = message->pose.pose;
    latest_pose_ = pose;
    latest_pose_received_ = std::chrono::steady_clock::now();
  }

  // Special function: Refresh guest insertion order and enforce the configured bound.
  void touch_guest(const std::string &id) {
    guest_order_.erase(std::remove(guest_order_.begin(), guest_order_.end(), id),
                       guest_order_.end());
    guest_order_.push_back(id);
    while (guest_order_.size() > max_guest_records_) {
      guest_records_.erase(guest_order_.front());
      guest_order_.pop_front();
    }
  }

  // Special function: Refresh find recency order and enforce the configured bound.
  void touch_found(const std::string &key) {
    find_order_.erase(std::remove(find_order_.begin(), find_order_.end(), key),
                      find_order_.end());
    find_order_.push_back(key);
    while (find_order_.size() > max_find_records_) {
      find_records_.erase(find_order_.front());
      find_order_.pop_front();
    }
  }

  std::mutex mutex_;
  const std::string session_id_;
  std::size_t max_find_records_{32};
  std::size_t max_guest_records_{32};
  double pose_max_age_sec_{1.0};
  std::string active_memory_id_;
  std::map<std::string, GuestRecord> guest_records_;
  std::deque<std::string> guest_order_;
  std::map<std::string, FoundRecord> find_records_;
  std::deque<std::string> find_order_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_pose_;
  std::chrono::steady_clock::time_point latest_pose_received_{};
  rclcpp::Service<robot_ai_interfaces::srv::SaveGpsrGuest>::SharedPtr save_service_;
  rclcpp::Service<robot_ai_interfaces::srv::GetGpsrGuest>::SharedPtr get_service_;
  rclcpp::Service<robot_ai_interfaces::srv::ListGpsrGuests>::SharedPtr list_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::Service<robot_ai_interfaces::srv::SaveFoundEntity>::SharedPtr
      find_save_service_;
  rclcpp::Service<robot_ai_interfaces::srv::LookupFoundEntity>::SharedPtr
      find_lookup_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr find_clear_service_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_subscription_;
};

}  // namespace robot_yasmin_tasks

int main(int argc, char **argv) {
  // Section: Run the process-owned RAM service until ROS shutdown.
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_yasmin_tasks::GpsrGuestMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
