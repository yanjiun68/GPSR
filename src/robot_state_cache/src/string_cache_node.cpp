// Section: Node implementation connects generic bounded storage to standard ROS
// topics.
#include "robot_state_cache/string_cache_node.hpp"

// Section: Standard utilities support numeric validation and diagnostic
// serialization.
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace robot_state_cache {

namespace {

diagnostic_msgs::msg::KeyValue key_value(const std::string &key,
                                         const std::string &value) {
  // Special function: Construct one explicit diagnostic field without
  // repetitive boilerplate.
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::chrono::steady_clock::duration
seconds_to_steady_duration(const double seconds) {
  // Special function: Convert validated seconds into the monotonic duration
  // used by cache state evaluation.
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(seconds));
}

} // namespace

StringCacheNode::StringCacheNode(const rclcpp::NodeOptions &options)
    : Node("robot_state_cache", options) {
  // Section: Declare stable topic, memory, timing, publication, and validation
  // policy.
  const auto input_topic =
      declare_parameter<std::string>("input_topic", "/robot_state_cache/input");
  const auto held_topic =
      declare_parameter<std::string>("held_topic", "/robot_state_cache/held");
  const auto status_topic = declare_parameter<std::string>(
      "status_topic", "/robot_state_cache/status");
  const auto cache_capacity =
      declare_parameter<std::int64_t>("cache_capacity", 10);
  const auto publish_rate_hz =
      declare_parameter<double>("publish_rate_hz", 20.0);
  const auto stale_timeout_sec =
      declare_parameter<double>("stale_timeout_sec", 0.5);
  const auto expire_timeout_sec =
      declare_parameter<double>("expire_timeout_sec", 2.0);
  publish_stale_data_ = declare_parameter<bool>("publish_stale_data", true);
  publish_expired_data_ =
      declare_parameter<bool>("publish_expired_data", false);
  reject_empty_input_ = declare_parameter<bool>("reject_empty_input", true);

  // Section: Fail startup with actionable errors rather than silently
  // correcting unsafe configuration.
  if (cache_capacity <= 0) {
    throw std::invalid_argument("cache_capacity must be greater than zero.");
  }
  if (!std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0) {
    throw std::invalid_argument(
        "publish_rate_hz must be finite and greater than zero.");
  }
  if (!std::isfinite(stale_timeout_sec) || stale_timeout_sec < 0.0) {
    throw std::invalid_argument(
        "stale_timeout_sec must be finite and non-negative.");
  }
  if (!std::isfinite(expire_timeout_sec) ||
      expire_timeout_sec < stale_timeout_sec) {
    throw std::invalid_argument("expire_timeout_sec must be finite and greater "
                                "than or equal to stale_timeout_sec.");
  }
  if (input_topic.empty() || held_topic.empty() || status_topic.empty()) {
    throw std::invalid_argument(
        "input_topic, held_topic, and status_topic must be non-empty.");
  }
  if (input_topic == held_topic) {
    throw std::invalid_argument(
        "input_topic and held_topic must differ to prevent a feedback loop.");
  }

  // Section: The validation hook rejects empty values without replacing the
  // previous accepted sample.
  const bool reject_empty = reject_empty_input_;
  cache_ = std::make_unique<CircularCache<HeldString>>(
      static_cast<std::size_t>(cache_capacity),
      [reject_empty](const HeldString &sample) {
        return !reject_empty || !sample.message.data.empty();
      });
  stale_timeout_ = seconds_to_steady_duration(stale_timeout_sec);
  expire_timeout_ = seconds_to_steady_duration(expire_timeout_sec);

  // Section: Input is a reliable event stream; held output is reliable
  // transient-local state for late consumers.
  auto input_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  input_qos.reliable().durability_volatile();
  auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  state_qos.reliable().transient_local();
  input_subscription_ = create_subscription<std_msgs::msg::String>(
      input_topic, input_qos,
      std::bind(&StringCacheNode::input_callback, this, std::placeholders::_1));
  held_publisher_ =
      create_publisher<std_msgs::msg::String>(held_topic, state_qos);
  status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      status_topic, state_qos);

  // Section: The wall timer reads held state only; it never writes the cache or
  // advances input sequence.
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));
  publish_timer_ = create_wall_timer(
      period, std::bind(&StringCacheNode::publish_timer_callback, this));

  RCLCPP_INFO(get_logger(),
              "State cache started: capacity=%ld publish_rate=%.3fHz "
              "stale=%.3fs expire=%.3fs input=%s held=%s",
              static_cast<long>(cache_capacity), publish_rate_hz,
              stale_timeout_sec, expire_timeout_sec, input_topic.c_str(),
              held_topic.c_str());
}

void StringCacheNode::input_callback(
    const std_msgs::msg::String::SharedPtr message) {
  // Special function: Timestamp and validate each producer event before one
  // bounded cache update.
  HeldString sample{*message, now()};
  const auto accepted = cache_->update(sample);
  if (!accepted.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Rejected cache input; the previous held value and "
                         "sequence remain unchanged.");
    return;
  }
  RCLCPP_DEBUG(get_logger(), "Accepted cache update sequence=%lu",
               accepted->sequence);
}

void StringCacheNode::publish_timer_callback() {
  // Special function: Snapshot under the cache mutex, then publish after the
  // mutex has been released.
  const auto snapshot = cache_->snapshot(
      CircularCache<HeldString>::Clock::now(), stale_timeout_, expire_timeout_);
  const auto publication_time = now();
  status_publisher_->publish(make_status_message(snapshot, publication_time));
  const double age_sec = snapshot.age.has_value() ? snapshot.age->count() : 0.0;
  log_state_transition(snapshot.status, age_sec);
  if (snapshot.latest.has_value() && should_publish_held(snapshot.status)) {
    held_publisher_->publish(snapshot.latest->value.message);
  }
}

diagnostic_msgs::msg::DiagnosticArray StringCacheNode::make_status_message(
    const CacheSnapshot<HeldString> &snapshot,
    const rclcpp::Time &publication_time) const {
  // Special function: Expose freshness and bounded-memory metadata through
  // standard diagnostics.
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = publication_time;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string(get_fully_qualified_name()) + "/cache";
  status.hardware_id = "robot_state_cache";
  status.message = to_string(snapshot.status);
  if (snapshot.status == CacheStatus::VALID) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  } else if (snapshot.status == CacheStatus::EXPIRED) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }
  const bool has_value = snapshot.latest.has_value();
  status.values.push_back(key_value("state", to_string(snapshot.status)));
  status.values.push_back(key_value("has_value", has_value ? "true" : "false"));
  status.values.push_back(
      key_value("is_currently_valid",
                snapshot.status == CacheStatus::VALID ? "true" : "false"));
  status.values.push_back(key_value(
      "sequence", has_value ? std::to_string(snapshot.latest->sequence) : "0"));
  status.values.push_back(key_value(
      "age_sec",
      snapshot.age.has_value() ? std::to_string(snapshot.age->count()) : "-1"));
  status.values.push_back(
      key_value("history_count", std::to_string(snapshot.history_count)));
  status.values.push_back(
      key_value("capacity", std::to_string(snapshot.capacity)));
  status.values.push_back(key_value(
      "accepted_ros_time_sec",
      has_value
          ? std::to_string(snapshot.latest->value.accepted_ros_time.seconds())
          : "-1"));
  array.status.push_back(std::move(status));
  return array;
}

bool StringCacheNode::should_publish_held(const CacheStatus status) const {
  // Special function: Apply safe output policy independently from preservation
  // of cache history.
  if (status == CacheStatus::VALID) {
    return true;
  }
  if (status == CacheStatus::STALE) {
    return publish_stale_data_;
  }
  if (status == CacheStatus::EXPIRED) {
    return publish_expired_data_;
  }
  return false;
}

void StringCacheNode::log_state_transition(const CacheStatus status,
                                           const double age_sec) {
  // Special function: Log freshness transitions once instead of producing
  // timer-rate warning spam.
  if (status == last_logged_status_) {
    return;
  }
  if (status == CacheStatus::STALE) {
    RCLCPP_WARN(get_logger(), "Cache data became STALE at age=%.3fs.", age_sec);
  } else if (status == CacheStatus::EXPIRED) {
    RCLCPP_WARN(get_logger(), "Cache data became EXPIRED at age=%.3fs.",
                age_sec);
  } else if (status == CacheStatus::VALID) {
    RCLCPP_INFO(get_logger(), "Cache data is VALID at age=%.3fs.", age_sec);
  } else {
    RCLCPP_INFO(get_logger(), "Cache is EMPTY.");
  }
  last_logged_status_ = status;
}

} // namespace robot_state_cache
