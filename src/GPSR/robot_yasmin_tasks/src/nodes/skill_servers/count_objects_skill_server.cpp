// Section: Lifecycle action server that converts object-quantity observations into
// count results.
#include <algorithm>
#include <atomic>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_ai_interfaces/action/count_objects.hpp>
#include <robot_ai_interfaces/srv/count_objects.hpp>
#include <string>
#include <thread>

#include "robot_yasmin_tasks/runtime.hpp"

namespace robot_yasmin_tasks {

// Section: Lifecycle action handling for object-count requests and observation
// aggregation.
class CountObjectsSkillServer final : public rclcpp_lifecycle::LifecycleNode {
 public:
  using Action = robot_ai_interfaces::action::CountObjects;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using Service = robot_ai_interfaces::srv::CountObjects;
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Special function: Declare stable interface names without allocating task resources.
  CountObjectsSkillServer()
      : rclcpp_lifecycle::LifecycleNode("count_objects_skill_server") {
    action_name_ = declare_parameter<std::string>("action_name", "/gpsr/count_objects");
    service_name_ = declare_parameter<std::string>("perception_service",
                                                   "/perception/count_objects");
    default_timeout_sec_ = declare_parameter<double>("default_timeout_sec", 5.0);
    RCLCPP_INFO(get_logger(), "CountObjects lifecycle server is UNCONFIGURED.");
  }

  // Special function: Create the same perception client and GPSR action server on
  // configure.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    try {
      count_client_ = create_client<Service>(service_name_);
      action_server_ = rclcpp_action::create_server<Action>(
          get_node_base_interface(), get_node_clock_interface(),
          get_node_logging_interface(), get_node_waitables_interface(), action_name_,
          std::bind(&CountObjectsSkillServer::goal_callback, this,
                    std::placeholders::_1, std::placeholders::_2),
          std::bind(&CountObjectsSkillServer::cancel_callback, this,
                    std::placeholders::_1),
          std::bind(&CountObjectsSkillServer::accepted_callback, this,
                    std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "[GPSR_LIFECYCLE] CountObjects: configured");
      return CallbackReturn::SUCCESS;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "CountObjects configure failed: %s", error.what());
      release_resources();
      return CallbackReturn::FAILURE;
    }
  }

  // Special function: Permit goals only in ACTIVE lifecycle state.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    skill_active_.store(true);
    RCLCPP_INFO(get_logger(), "[GPSR_LIFECYCLE] CountObjects: ACTIVE");
    return CallbackReturn::SUCCESS;
  }

  // Special function: Reject new work after deactivation.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    skill_active_.store(false);
    return CallbackReturn::SUCCESS;
  }

  // Special function: Release configure-owned interfaces on cleanup.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    skill_active_.store(false);
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  // Special function: Release configure-owned interfaces on shutdown.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    skill_active_.store(false);
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  // Special function: Fail closed and restore the unconfigured resource baseline.
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override {
    skill_active_.store(false);
    release_resources();
    return CallbackReturn::SUCCESS;
  }

 private:
  // Special function: Destroy all resources allocated during configure.
  void release_resources() {
    action_server_.reset();
    count_client_.reset();
  }

  // Special function: Reject inactive and negative-timeout goals.
  rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &,
                                            std::shared_ptr<const Action::Goal> goal) {
    if (!skill_active_.load() || goal->timeout_sec < 0.0F) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Special function: Accept queue-driven cancellation.
  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Special function: Execute accepted work outside the action-server callback.
  void accepted_callback(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  // Special function: Forward one action request to the existing perception service.
  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Action::Result>();
    if (!skill_active_.load() || !count_client_) {
      result->success = false;
      result->result_json = "{}";
      result->message = "CountObjects lifecycle node is not ACTIVE.";
      goal_handle->abort(result);
      return;
    }
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->status = "Counting " + goal->category + " at " +
                       (goal->location.empty() ? "current view" : goal->location);
    goal_handle->publish_feedback(feedback);
    const double timeout =
        goal->timeout_sec != 0.0F ? goal->timeout_sec : default_timeout_sec_;
    if (!count_client_->wait_for_service(std::chrono::duration<double>(timeout))) {
      result->success = false;
      result->result_json = "{}";
      result->message = "Perception count_objects service is unavailable.";
      goal_handle->abort(result);
      return;
    }
    auto request = std::make_shared<Service::Request>();
    request->category = goal->category;
    request->location = goal->location;
    const auto filter = parse_json(goal->filter_json, Json::object());
    request->filter_json = filter.is_object() ? dump_json(filter) : "{}";
    auto future = count_client_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(timeout)) !=
        std::future_status::ready) {
      result->success = false;
      result->result_json = "{}";
      result->message = "Perception count_objects service is unavailable.";
      goal_handle->abort(result);
      return;
    }
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->result_json = "{}";
      result->message = "CountObjects goal was cancelled.";
      goal_handle->canceled(result);
      return;
    }
    const auto response = future.get();
    result->success = response->success;
    result->count = response->count;
    result->result_json = response->result_json;
    result->message = response->message;
    response->success ? goal_handle->succeed(result) : goal_handle->abort(result);
  }

  std::string action_name_;
  std::string service_name_;
  double default_timeout_sec_{5.0};
  std::atomic_bool skill_active_{false};
  rclcpp::Client<Service>::SharedPtr count_client_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
};

}  // namespace robot_yasmin_tasks

// Section: Process startup and lifecycle action-server execution.
int main(int argc, char **argv) {
  // Section: Preserve the four-thread executor used by the Python server.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_yasmin_tasks::CountObjectsSkillServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
