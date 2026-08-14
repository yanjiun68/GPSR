#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <smacc2/smacc.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>

#include "carry_my_luggage_phase1/retry_policy.hpp"
#include "carry_my_luggage_phase1/sm_carry_my_luggage_phase1.hpp"
#include "carry_my_luggage_phase1/utils/bringup_shutdown.hpp"

namespace carry_my_luggage_phase1
{
using namespace std::chrono_literals;

struct StAborted : smacc2::SmaccState<StAborted, SmCarryMyLuggagePhase1>
{
  using SmaccState::SmaccState;

  static void staticConfigure() {}

  static bool isArmInRestPose(
    const sensor_msgs::msg::JointState & joint_state, rclcpp::Logger logger)
  {
    constexpr std::array<const char *, 5> kArmJointNames = {
      "waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"
    };
    constexpr std::array<double, 5> kSleepJoints = {-1.57, -1.8, 1.55, 0.8, 0.0};
    constexpr double kJointToleranceRad = 0.2;

    for (std::size_t i = 0; i < kArmJointNames.size(); ++i) {
      const auto * joint_name = kArmJointNames[i];
      const auto target = kSleepJoints[i];

      const auto it = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
      if (it == joint_state.name.end()) {
        RCLCPP_WARN(
          logger,
          "Joint state does not contain '%s'; cannot confirm arm rest pose.",
          joint_name);
        return false;
      }

      const auto index = static_cast<std::size_t>(std::distance(joint_state.name.begin(), it));
      if (index >= joint_state.position.size()) {
        RCLCPP_WARN(
          logger,
          "Joint state position array is missing '%s'; cannot confirm arm rest pose.",
          joint_name);
        return false;
      }

      const auto error = std::fabs(joint_state.position[index] - target);
      if (error > kJointToleranceRad) {
        return false;
      }
    }

    return true;
  }

  void tryRestArmBeforeShutdown()
  {
    constexpr auto kJointStateTimeout = std::chrono::milliseconds(1500);
    constexpr auto kServiceTimeout = std::chrono::seconds(2);
    constexpr auto kRestResponseTimeout = std::chrono::seconds(45);
    constexpr auto kRetryDelay = std::chrono::seconds(1);
    constexpr int kMaxRestAttempts = 3;

    auto helper_node = std::make_shared<rclcpp::Node>("st_aborted_arm_rest_helper");
    sensor_msgs::msg::JointState joint_state;
    const bool got_joint_state =
      rclcpp::wait_for_message(joint_state, helper_node, "/joint_states", kJointStateTimeout);

    if (got_joint_state && isArmInRestPose(joint_state, getLogger())) {
      RCLCPP_INFO(getLogger(), "Arm is already in rest pose. Skipping /grasp/rest.");
      return;
    }

    if (got_joint_state) {
      RCLCPP_WARN(
        getLogger(),
        "Arm is not in rest pose. Calling /grasp/rest before shutting down heavy nodes.");
    } else {
      RCLCPP_WARN(
        getLogger(),
        "Timed out waiting for /joint_states in StAborted. Calling /grasp/rest as a precaution.");
    }

    auto client = helper_node->create_client<std_srvs::srv::Trigger>("/grasp/rest");
    for (int attempt = 1; attempt <= kMaxRestAttempts; ++attempt) {
      if (!client->wait_for_service(kServiceTimeout)) {
        if (attempt < kMaxRestAttempts) {
          RCLCPP_WARN(
            getLogger(),
            "/grasp/rest service is not available in StAborted (attempt %d/%d). Retrying.",
            attempt, kMaxRestAttempts);
          rclcpp::sleep_for(kRetryDelay);
          continue;
        }

        RCLCPP_ERROR(
          getLogger(),
          "/grasp/rest service was unavailable for %d attempts in StAborted. "
          "Manual interruption may be required. If the arm remains unsafe, interrupt frequently within 5 seconds while shutdown continues.",
          kMaxRestAttempts);
        return;
      }

      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future = client->async_send_request(request);
      const auto result =
        rclcpp::spin_until_future_complete(helper_node, future, kRestResponseTimeout);

      if (result != rclcpp::FutureReturnCode::SUCCESS) {
        if (attempt < kMaxRestAttempts) {
          RCLCPP_WARN(
            getLogger(),
            "Timed out waiting for /grasp/rest in StAborted (attempt %d/%d). Retrying.",
            attempt, kMaxRestAttempts);
          rclcpp::sleep_for(kRetryDelay);
          continue;
        }

        RCLCPP_ERROR(
          getLogger(),
          "/grasp/rest timed out %d times in StAborted. Manual interruption may be required. "
          "If the arm remains unsafe, interrupt frequently within 5 seconds while shutdown continues.",
          kMaxRestAttempts);
        return;
      }

      const auto response = future.get();
      if (response == nullptr) {
        if (attempt < kMaxRestAttempts) {
          RCLCPP_WARN(
            getLogger(),
            "/grasp/rest returned a null response in StAborted (attempt %d/%d). Retrying.",
            attempt, kMaxRestAttempts);
          rclcpp::sleep_for(kRetryDelay);
          continue;
        }

        RCLCPP_ERROR(
          getLogger(),
          "/grasp/rest returned null responses for %d attempts in StAborted. "
          "Manual interruption may be required. If the arm remains unsafe, interrupt frequently within 5 seconds while shutdown continues.",
          kMaxRestAttempts);
        return;
      }

      if (!response->success) {
        if (attempt < kMaxRestAttempts) {
          RCLCPP_WARN(
            getLogger(),
            "/grasp/rest failed in StAborted (attempt %d/%d): %s. Retrying.",
            attempt, kMaxRestAttempts, response->message.c_str());
          rclcpp::sleep_for(kRetryDelay);
          continue;
        }

        RCLCPP_ERROR(
          getLogger(),
          "/grasp/rest failed %d times in StAborted: %s. Manual interruption may be required. "
          "If the arm remains unsafe, interrupt frequently within 5 seconds while shutdown continues.",
          kMaxRestAttempts, response->message.c_str());
        return;
      }

      RCLCPP_INFO(
        getLogger(),
        "/grasp/rest succeeded in StAborted on attempt %d/%d: %s",
        attempt, kMaxRestAttempts, response->message.c_str());
      return;
    }
  }

  void onEntry()
  {
    int retry_count = 0;
    getStateMachine().getGlobalSMData("retry_count", retry_count);
    RCLCPP_ERROR(
      getLogger(),
      "Retry limit exceeded (%d > %d). Aborting carry_my_luggage_phase1 state machine.",
      retry_count, kMaxRetryAttempts);

    tryRestArmBeforeShutdown();

    while (rclcpp::ok() && !utils::shutdownHeavyBringupNodes(this)) {
      RCLCPP_ERROR(
        getLogger(),
        "Heavy nodes are still running. Retrying shutdown before ending StAborted.");
      rclcpp::sleep_for(500ms);
    }

    getStateMachine().stop();
  }
};
}  // namespace carry_my_luggage_phase1
