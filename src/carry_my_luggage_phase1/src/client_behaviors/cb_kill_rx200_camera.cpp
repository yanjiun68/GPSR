#include "carry_my_luggage_phase1/client_behaviors/cb_kill_rx200_camera.hpp"

#include <putrabot_interfaces/srv/kill_managed_process.hpp>

#include <chrono>
#include <future>
#include <memory>

using namespace std::chrono_literals;

namespace
{
constexpr auto kServiceTimeout = std::chrono::seconds(5);
constexpr auto kResponseTimeout = std::chrono::seconds(5);
constexpr auto kPollInterval = 100ms;
constexpr char kKillService[] = "/kill";
constexpr char kRx200CameraTarget[] = "/camera/rx200_camera";
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbKillRx200Camera::onEntry()
{
  auto node = getNode();
  auto client = node->create_client<putrabot_interfaces::srv::KillManagedProcess>(kKillService);

  if (!client->wait_for_service(kServiceTimeout)) {
    RCLCPP_WARN(
      getLogger(),
      "%s service is not available; continuing without killing %s.",
      kKillService,
      kRx200CameraTarget);
    postSuccessEvent();
    return;
  }

  auto request = std::make_shared<putrabot_interfaces::srv::KillManagedProcess::Request>();
  request->target = kRx200CameraTarget;

  RCLCPP_INFO(getLogger(), "Requesting %s to stop %s.", kKillService, kRx200CameraTarget);
  auto future = client->async_send_request(request);
  const auto start = std::chrono::steady_clock::now();

  while (
    !isShutdownRequested() && rclcpp::ok() &&
    future.wait_for(kPollInterval) != std::future_status::ready)
  {
    if ((std::chrono::steady_clock::now() - start) > kResponseTimeout) {
      RCLCPP_WARN(
        getLogger(),
        "Timed out waiting for %s response while killing %s; continuing.",
        kKillService,
        kRx200CameraTarget);
      postSuccessEvent();
      return;
    }
    rclcpp::sleep_for(kPollInterval);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postSuccessEvent();
    return;
  }

  const auto response = future.get();
  if (response == nullptr) {
    RCLCPP_WARN(
      getLogger(),
      "%s returned a null response while killing %s; continuing.",
      kKillService,
      kRx200CameraTarget);
    postSuccessEvent();
    return;
  }

  if (response->success) {
    RCLCPP_INFO(
      getLogger(),
      "%s stopped %s: %s",
      kKillService,
      kRx200CameraTarget,
      response->message.c_str());
  } else {
    RCLCPP_WARN(
      getLogger(),
      "%s did not stop %s: %s",
      kKillService,
      kRx200CameraTarget,
      response->message.c_str());
  }

  postSuccessEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
