#include "carry_my_luggage_phase1/client_behaviors/cb_breadcrumb.hpp"

#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <future>

using namespace std::chrono_literals;

namespace
{
constexpr char kBreadcrumbStartTeachService[] = "/breadcrumb_teach_repeat/start_teach";
constexpr auto kServiceTimeout = std::chrono::seconds(10);
constexpr auto kResponseTimeout = std::chrono::seconds(15);
constexpr auto kRetryDelay = std::chrono::seconds(1);
}  // namespace

namespace carry_my_luggage_phase1::client_behaviors
{
void CbBreadCrumb::onEntry()
{
  auto node = getNode();
  auto client = node->create_client<std_srvs::srv::Trigger>(kBreadcrumbStartTeachService);

  if (!client->wait_for_service(kServiceTimeout)) {
    RCLCPP_WARN(getLogger(), "%s service is not available.", kBreadcrumbStartTeachService);
    rclcpp::sleep_for(kRetryDelay);
    postFailureEvent();
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  const auto start = std::chrono::steady_clock::now();
  while (!isShutdownRequested() && rclcpp::ok() && future.wait_for(100ms) != std::future_status::ready) {
    if ((std::chrono::steady_clock::now() - start) > kResponseTimeout) {
      RCLCPP_WARN(getLogger(), "Timed out waiting for %s.", kBreadcrumbStartTeachService);
      rclcpp::sleep_for(kRetryDelay);
      postFailureEvent();
      return;
    }
    rclcpp::sleep_for(100ms);
  }

  if (!rclcpp::ok() || isShutdownRequested()) {
    postFailureEvent();
    return;
  }

  const auto response = future.get();
  if (response == nullptr) {
    RCLCPP_WARN(getLogger(), "%s returned a null response.", kBreadcrumbStartTeachService);
    rclcpp::sleep_for(kRetryDelay);
    postFailureEvent();
    return;
  }

  if (response->success) {
    RCLCPP_INFO(getLogger(), "%s succeeded: %s", kBreadcrumbStartTeachService, response->message.c_str());
    postSuccessEvent();
    return;
  }

  RCLCPP_WARN(getLogger(), "%s failed: %s", kBreadcrumbStartTeachService, response->message.c_str());
  rclcpp::sleep_for(kRetryDelay);
  postFailureEvent();
}
}  // namespace carry_my_luggage_phase1::client_behaviors
