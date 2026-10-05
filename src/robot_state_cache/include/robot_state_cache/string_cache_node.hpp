#ifndef ROBOT_STATE_CACHE__STRING_CACHE_NODE_HPP_
#define ROBOT_STATE_CACHE__STRING_CACHE_NODE_HPP_

// Section: Standard ownership and timing support the bounded cache and
// fixed-rate timer.
#include <chrono>
#include <memory>
#include <string>

// Section: ROS interfaces expose a reusable reference adapter without adding
// custom messages.
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "robot_state_cache/cache_status.hpp"
#include "robot_state_cache/circular_cache.hpp"

namespace robot_state_cache {

struct HeldString {
  // Section: Preserve the original value and the ROS acceptance time as one
  // cache payload.
  std_msgs::msg::String message;
  rclcpp::Time accepted_ros_time;
};

class StringCacheNode : public rclcpp::Node {
public:
  explicit StringCacheNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void input_callback(const std_msgs::msg::String::SharedPtr message);
  void publish_timer_callback();
  diagnostic_msgs::msg::DiagnosticArray
  make_status_message(const CacheSnapshot<HeldString> &snapshot,
                      const rclcpp::Time &publication_time) const;
  bool should_publish_held(CacheStatus status) const;
  void log_state_transition(CacheStatus status, double age_sec);

  // Section: Cache and timing policy are immutable after validated node
  // construction.
  std::unique_ptr<CircularCache<HeldString>> cache_;
  std::chrono::steady_clock::duration stale_timeout_;
  std::chrono::steady_clock::duration expire_timeout_;
  bool publish_stale_data_{true};
  bool publish_expired_data_{false};
  bool reject_empty_input_{true};
  CacheStatus last_logged_status_{CacheStatus::EMPTY};

  // Section: ROS entities keep producer callbacks independent from
  // timer-controlled consumers.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr input_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr held_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      status_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

} // namespace robot_state_cache

#endif // ROBOT_STATE_CACHE__STRING_CACHE_NODE_HPP_
