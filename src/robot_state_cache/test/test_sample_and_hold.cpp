// Section: ROS integration test verifies independent producer and held-output
// timing.
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "robot_state_cache/string_cache_node.hpp"

namespace {

using namespace std::chrono_literals;

class SampleAndHoldIntegration : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    // Special function: Initialize one ROS context for the integration fixture
    // process.
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    // Special function: Shut down DDS resources after all timer and callback
    // assertions complete.
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

std::map<std::string, std::string>
diagnostic_values(const diagnostic_msgs::msg::DiagnosticArray &message) {
  // Special function: Convert standard diagnostic key/value fields into
  // deterministic test lookup state.
  std::map<std::string, std::string> values;
  if (message.status.empty()) {
    return values;
  }
  for (const auto &item : message.status.front().values) {
    values[item.key] = item.value;
  }
  return values;
}

TEST_F(SampleAndHoldIntegration,
       TimerRepeatsHeldValueWithoutAdvancingSequence) {
  // Section: Short thresholds keep the complete VALID/STALE/EXPIRED/recovery
  // scenario bounded.
  rclcpp::NodeOptions options;
  options.append_parameter_override("input_topic", "/test_cache/input");
  options.append_parameter_override("held_topic", "/test_cache/held");
  options.append_parameter_override("status_topic", "/test_cache/status");
  options.append_parameter_override("cache_capacity", 5);
  options.append_parameter_override("publish_rate_hz", 20.0);
  options.append_parameter_override("stale_timeout_sec", 0.20);
  options.append_parameter_override("expire_timeout_sec", 0.45);
  options.append_parameter_override("publish_stale_data", true);
  options.append_parameter_override("publish_expired_data", false);
  auto cache_node =
      std::make_shared<robot_state_cache::StringCacheNode>(options);
  auto test_node =
      std::make_shared<rclcpp::Node>("sample_and_hold_test_driver");

  // Section: Test endpoints match production reliable input and transient-local
  // held state QoS.
  auto input_qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto state_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto input_publisher = test_node->create_publisher<std_msgs::msg::String>(
      "/test_cache/input", input_qos);
  std::mutex observation_mutex;
  std::vector<std::string> held_values;
  std::vector<std::map<std::string, std::string>> statuses;
  auto held_subscription =
      test_node->create_subscription<std_msgs::msg::String>(
          "/test_cache/held", state_qos,
          [&observation_mutex,
           &held_values](const std_msgs::msg::String::SharedPtr message) {
            std::lock_guard<std::mutex> lock(observation_mutex);
            held_values.push_back(message->data);
          });
  auto status_subscription =
      test_node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
          "/test_cache/status", state_qos,
          [&observation_mutex, &statuses](
              const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
            std::lock_guard<std::mutex> lock(observation_mutex);
            statuses.push_back(diagnostic_values(*message));
          });

  // Section: A multithreaded executor exercises concurrent subscription and
  // timer access.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2U);
  executor.add_node(cache_node);
  executor.add_node(test_node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  const auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
  while (input_publisher->get_subscription_count() == 0U &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(input_publisher->get_subscription_count(), 0U);

  // Section: One producer update must yield repeated 20 Hz output with one
  // unchanged sequence.
  std_msgs::msg::String input;
  input.data = "ROI_A";
  input_publisher->publish(input);
  std::this_thread::sleep_for(330ms);
  {
    std::lock_guard<std::mutex> lock(observation_mutex);
    ASSERT_GE(held_values.size(), 4U);
    for (const auto &value : held_values) {
      EXPECT_EQ(value, "ROI_A");
    }
    bool saw_valid_sequence_one = false;
    bool saw_stale_sequence_one = false;
    for (const auto &status : statuses) {
      if (status.count("sequence") > 0U && status.at("sequence") == "1") {
        saw_valid_sequence_one |= status.at("state") == "VALID";
        saw_stale_sequence_one |= status.at("state") == "STALE";
      }
    }
    EXPECT_TRUE(saw_valid_sequence_one);
    EXPECT_TRUE(saw_stale_sequence_one);
  }

  // Section: Expiration remains observable in status while safe policy stops
  // held-value output.
  std::this_thread::sleep_for(220ms);
  std::size_t held_count_at_expiry = 0U;
  {
    std::lock_guard<std::mutex> lock(observation_mutex);
    held_count_at_expiry = held_values.size();
    ASSERT_FALSE(statuses.empty());
    EXPECT_EQ(statuses.back().at("state"), "EXPIRED");
    EXPECT_EQ(statuses.back().at("sequence"), "1");
    EXPECT_EQ(statuses.back().at("has_value"), "true");
    EXPECT_EQ(statuses.back().at("is_currently_valid"), "false");
  }
  std::this_thread::sleep_for(150ms);
  {
    std::lock_guard<std::mutex> lock(observation_mutex);
    EXPECT_EQ(held_values.size(), held_count_at_expiry);
  }

  // Section: A new update recovers EXPIRED to VALID and increments sequence
  // exactly once.
  input.data = "ROI_B";
  input_publisher->publish(input);
  std::this_thread::sleep_for(140ms);
  {
    std::lock_guard<std::mutex> lock(observation_mutex);
    ASSERT_FALSE(held_values.empty());
    EXPECT_EQ(held_values.back(), "ROI_B");
    ASSERT_FALSE(statuses.empty());
    EXPECT_EQ(statuses.back().at("state"), "VALID");
    EXPECT_EQ(statuses.back().at("sequence"), "2");
    EXPECT_EQ(statuses.back().at("history_count"), "2");
    EXPECT_EQ(statuses.back().at("capacity"), "5");
  }

  // Section: Stop callbacks before fixtures and captured observation storage
  // are destroyed.
  executor.cancel();
  spin_thread.join();
  executor.remove_node(test_node);
  executor.remove_node(cache_node);
  held_subscription.reset();
  status_subscription.reset();
  input_publisher.reset();
  test_node.reset();
  cache_node.reset();
}

TEST_F(SampleAndHoldIntegration, InvalidConfigurationFailsAtStartup) {
  // Section: Reversed stale/expiry thresholds must fail instead of being
  // silently clamped.
  rclcpp::NodeOptions options;
  options.append_parameter_override("stale_timeout_sec", 2.0);
  options.append_parameter_override("expire_timeout_sec", 1.0);
  EXPECT_THROW(robot_state_cache::StringCacheNode node(options),
               std::invalid_argument);
}

} // namespace
