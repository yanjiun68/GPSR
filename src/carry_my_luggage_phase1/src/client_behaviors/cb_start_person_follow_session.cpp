#include "carry_my_luggage_phase1/client_behaviors/cb_start_person_follow_session.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <person_follower_master/msg/master_target.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <voice_interfaces/srv/say_text.hpp>

#include "carry_my_luggage_phase1/client_behaviors/cb_person_following_driver.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr auto kBringupReadyTimeout = std::chrono::seconds(40);
constexpr auto kMasterTargetTimeout = std::chrono::seconds(1);
constexpr auto kTargetStillHoldDuration = std::chrono::seconds(5);
constexpr auto kTtsServiceTimeout = std::chrono::seconds(5);
constexpr double kTargetStillMaxBearingRad = 0.30;
constexpr double kTargetStillMinDistanceM = 0.80;
constexpr double kTargetStillMaxDistanceM = 2.00;
constexpr double kTargetStillMaxSpeedMps = 0.12;
constexpr char kMasterTargetTopic[] = "/master_target";
constexpr char kTargetPrompt[] = "Please stand in front of me so I can follow you";

struct TargetMonitorState
{
  bool target_recent = false;
  std::chrono::steady_clock::time_point last_target_at{};
  std::chrono::steady_clock::time_point still_since{};
};

bool isZeroTimePoint(const std::chrono::steady_clock::time_point& time_point)
{
  return time_point.time_since_epoch().count() == 0;
}

bool isTargetStillGesture(const person_follower_master::msg::MasterTarget& msg)
{
  const bool healthy_target = msg.is_locked && msg.state != "LOST";
  const bool centered = std::abs(msg.bearing) <= kTargetStillMaxBearingRad;
  const bool in_range =
      msg.distance >= kTargetStillMinDistanceM && msg.distance <= kTargetStillMaxDistanceM;
  const bool still = std::hypot(msg.vx, msg.vy) <= kTargetStillMaxSpeedMps;
  return healthy_target && centered && in_range && still;
}
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbStartPersonFollowSession::onEntry()
{
  using SayText = voice_interfaces::srv::SayText;

  auto node = getNode();

  const auto bringup_ready_start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && !isShutdownRequested())
  {
    bool person_follow_bringup_ready = false;
    if (getStateMachine()->getGlobalSMData(
            kPersonFollowBringupReadyKey, person_follow_bringup_ready) &&
        person_follow_bringup_ready)
    {
      RCLCPP_INFO(getLogger(),
                  "CbPersonFollowingDriver succeeded. Person-follow stack is ready.");
      break;
    }

    if ((std::chrono::steady_clock::now() - bringup_ready_start) > kBringupReadyTimeout)
    {
      RCLCPP_WARN(getLogger(),
                  "Timed out waiting for CbPersonFollowingDriver success before entering "
                  "person-follow session.");
      postFailureEvent();
      return;
    }

    RCLCPP_INFO_THROTTLE(getLogger(), *node->get_clock(), 2000,
                         "Waiting for CbPersonFollowingDriver success before entering "
                         "person-follow session");
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested())
  {
    postFailureEvent();
    return;
  }

  auto tts_client = node->create_client<SayText>("/voice/tts/say");
  auto target_state = std::make_shared<TargetMonitorState>();
  auto target_mutex = std::make_shared<std::mutex>();
  auto master_target_sub =
      node->create_subscription<person_follower_master::msg::MasterTarget>(
          kMasterTargetTopic, rclcpp::QoS(1).reliable().transient_local(),
          [logger = getLogger(), target_state, target_mutex](
              const person_follower_master::msg::MasterTarget::SharedPtr msg)
          {
            if (msg == nullptr)
            {
              return;
            }

            const auto now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(*target_mutex);
            target_state->target_recent = true;
            target_state->last_target_at = now;

            if (isTargetStillGesture(*msg))
            {
              if (isZeroTimePoint(target_state->still_since))
              {
                target_state->still_since = now;
                RCLCPP_INFO(logger,
                            "Detected operator still gesture candidate on %s. "
                            "Holding for %lld seconds before stopping follow session.",
                            kMasterTargetTopic,
                            static_cast<long long>(kTargetStillHoldDuration.count()));
              }
            }
            else if (!isZeroTimePoint(target_state->still_since))
            {
              target_state->still_since = std::chrono::steady_clock::time_point{};
              RCLCPP_INFO(logger,
                          "Operator moved again; resetting still-gesture hold timer.");
            }
          });
  (void)master_target_sub;

  auto wait_for_future = [this](auto& future)
  {
    while (!isShutdownRequested() && rclcpp::ok() &&
           future.wait_for(100ms) != std::future_status::ready)
    {
      rclcpp::sleep_for(100ms);
    }

    return rclcpp::ok() && !isShutdownRequested();
  };

  auto prompt_target_alignment = [&]() -> bool
  {
    if (!tts_client->wait_for_service(kTtsServiceTimeout))
    {
      RCLCPP_WARN(getLogger(), "/voice/tts/say service is not available.");
      return false;
    }

    auto request = std::make_shared<SayText::Request>();
    request->text = kTargetPrompt;

    auto response_future = tts_client->async_send_request(request);
    if (!wait_for_future(response_future))
    {
      return false;
    }

    auto response = response_future.get();
    if (response == nullptr)
    {
      RCLCPP_WARN(getLogger(), "/voice/tts/say returned an empty response.");
      return false;
    }

    if (!response->success)
    {
      RCLCPP_WARN(getLogger(), "/voice/tts/say failed while prompting for alignment: %s",
                  response->message.c_str());
      return false;
    }

    RCLCPP_INFO(getLogger(), "Prompted operator to stand in front of the robot.");
    return true;
  };

  (void)prompt_target_alignment();

  while (!isShutdownRequested() && rclcpp::ok())
  {
    bool target_still_held_long_enough = false;
    {
      const auto now = std::chrono::steady_clock::now();
      std::lock_guard<std::mutex> lock(*target_mutex);
      if (target_state->target_recent &&
          (now - target_state->last_target_at) > kMasterTargetTimeout)
      {
        target_state->target_recent = false;
        target_state->still_since = std::chrono::steady_clock::time_point{};
        RCLCPP_WARN(getLogger(),
                    "Lost recent %s updates while waiting for still gesture; resetting hold timer.",
                    kMasterTargetTopic);
      }

      if (target_state->target_recent && !isZeroTimePoint(target_state->still_since))
      {
        target_still_held_long_enough =
            (now - target_state->still_since) >= kTargetStillHoldDuration;
      }
    }

    if (target_still_held_long_enough)
    {
      RCLCPP_INFO(getLogger(),
                  "Operator stayed still in front of the robot for %lld seconds. "
                  "Posting success; state exit will stop person-follow bringup.",
                  static_cast<long long>(kTargetStillHoldDuration.count()));
      postSuccessEvent();
      return;
    }

    RCLCPP_INFO_THROTTLE(
        getLogger(), *node->get_clock(), 2000,
        "Waiting for operator to stop in front of the robot for %lld seconds.",
        static_cast<long long>(kTargetStillHoldDuration.count()));
    rclcpp::sleep_for(100ms);
  }

  postFailureEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
