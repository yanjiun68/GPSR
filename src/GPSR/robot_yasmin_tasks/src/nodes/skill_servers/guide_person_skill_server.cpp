// Section: Lifecycle-managed sequential GUIDE_PERSON action server.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_ai_interfaces/action/guide_person.hpp>
#include <robot_ai_interfaces/action/navigate_named_pose.hpp>
#include <string>
#include <thread>
#include <voice_interfaces/srv/say_text.hpp>

namespace robot_yasmin_tasks {

// Section: Stable aliases keep the four guide states readable.
using GuideAction = robot_ai_interfaces::action::GuidePerson;
using GuideGoalHandle = rclcpp_action::ServerGoalHandle<GuideAction>;
using NavigateAction = robot_ai_interfaces::action::NavigateNamedPose;

class GuidePersonServer final : public rclcpp_lifecycle::LifecycleNode {
 public:
  // Special function: Declare the bounded guide-state policy in one configuration point.
  GuidePersonServer() : LifecycleNode("guide_person_skill_server") {
    declare_parameter<std::string>("action_name", "/person_guide/control");
    declare_parameter<std::string>("navigation_action", "/nav/navigate_named_pose");
    declare_parameter<std::string>("tts_service", "/voice/tts/say");
    declare_parameter<double>("default_timeout_sec", 120.0);
    declare_parameter<double>("goal_acceptance_timeout_sec", 5.0);
    declare_parameter<double>("speech_timeout_sec", 5.0);
    declare_parameter<std::int64_t>("max_navigation_retries", 1);
  }

  // Special function: Create action clients and the public GUIDE_PERSON server.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    default_timeout_sec_ = std::max(1.0, get_parameter("default_timeout_sec").as_double());
    goal_acceptance_timeout_sec_ =
        std::max(0.1, get_parameter("goal_acceptance_timeout_sec").as_double());
    speech_timeout_sec_ = std::max(0.1, get_parameter("speech_timeout_sec").as_double());
    max_navigation_retries_ = static_cast<std::uint32_t>(std::max<std::int64_t>(
        0, get_parameter("max_navigation_retries").as_int()));
    navigation_client_ = rclcpp_action::create_client<NavigateAction>(
        this, get_parameter("navigation_action").as_string());
    speech_client_ = create_client<voice_interfaces::srv::SayText>(
        get_parameter("tts_service").as_string());
    action_server_ = rclcpp_action::create_server<GuideAction>(
        this, get_parameter("action_name").as_string(),
        std::bind(&GuidePersonServer::admit, this, std::placeholders::_1,
                  std::placeholders::_2),
        [this](const std::shared_ptr<GuideGoalHandle>) {
          return busy_.load() ? rclcpp_action::CancelResponse::ACCEPT
                              : rclcpp_action::CancelResponse::REJECT;
        },
        std::bind(&GuidePersonServer::execute_async, this, std::placeholders::_1));
    return CallbackReturn::SUCCESS;
  }

  // Special function: Enable new guide requests only while lifecycle ACTIVE.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    active_.store(true);
    return CallbackReturn::SUCCESS;
  }

  // Special function: Reject new work while allowing an active goal to finish or cancel.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    active_.store(false);
    return busy_.load() ? CallbackReturn::FAILURE : CallbackReturn::SUCCESS;
  }

  // Special function: Release all ROS handles only after guide execution exits.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    if (busy_.load()) return CallbackReturn::FAILURE;
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  // Special function: Fail closed and drop every handle during shutdown.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    active_.store(false);
    release_resources();
    return CallbackReturn::SUCCESS;
  }

 private:
  // Special function: Validate both human and destination before reserving the server.
  rclcpp_action::GoalResponse admit(
      const rclcpp_action::GoalUUID &,
      const std::shared_ptr<const GuideAction::Goal> goal) {
    if (!active_.load() || goal->target_person.empty() || goal->destination.empty())
      return rclcpp_action::GoalResponse::REJECT;
    if (busy_.exchange(true)) return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Special function: Run the blocking four-state guide flow off executor callbacks.
  void execute_async(const std::shared_ptr<GuideGoalHandle> goal_handle) {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  // Special function: Execute S0 through S3 in order and commit success only after arrival.
  void execute(const std::shared_ptr<GuideGoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();
    const auto timeout = goal->timeout_sec > 0.0F ? goal->timeout_sec : default_timeout_sec_;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(timeout));

    // Section: S0 accepts the actual named Nav2 goal before any follow instruction.
    publish_feedback(goal_handle, "S0_CHECKING_NAVIGATION_GOAL");
    auto navigation_goal = send_navigation_goal(goal->destination, deadline, goal_handle);
    if (!navigation_goal) {
      say("I am not able to guide you to " + goal->destination +
          ". Please provide another location.");
      finish_failure(goal_handle, "NAVIGATION_GOAL_REJECTED: destination was not accepted.");
      return;
    }

    // Section: S1 is emitted strictly after navigation has accepted its goal.
    publish_feedback(goal_handle, "S1_REQUESTING_FOLLOW");
    if (!say("Please follow me along.")) {
      navigation_client_->async_cancel_goal(*navigation_goal);
      finish_failure(goal_handle, "SPEECH_FAILED: unable to issue the follow instruction.");
      return;
    }

    // Section: S2 waits for physical arrival and repeats S1 before each bounded retry.
    for (std::uint32_t attempt = 0; ; ++attempt) {
      publish_feedback(goal_handle, "S2_NAVIGATING");
      const auto navigation_outcome = wait_navigation_result(
          *navigation_goal, deadline, goal_handle);
      if (navigation_outcome == NavigationOutcome::ARRIVED) {
        // Section: S3 is reachable only after Nav2 reports successful arrival.
        publish_feedback(goal_handle, "S3_ANNOUNCING_ARRIVAL");
        if (!say("You have arrived at the destination.")) {
          finish_failure(goal_handle, "SPEECH_FAILED: arrival announcement failed.");
          return;
        }
        finish_success(goal_handle, "Guidance completed after arriving at the destination.");
        return;
      }
      if (navigation_outcome == NavigationOutcome::CANCELED) {
        finish_canceled(goal_handle);
        return;
      }
      if (attempt >= max_navigation_retries_ || std::chrono::steady_clock::now() >= deadline) {
        say("I am not able to guide you to the location.");
        finish_failure(goal_handle, "NAVIGATION_FAILED: destination was not reached.");
        return;
      }
      // Section: A failed navigation restarts from S1 before submitting its next goal.
      publish_feedback(goal_handle, "S1_REQUESTING_FOLLOW_RETRY");
      if (!say("Please follow me along.")) {
        finish_failure(goal_handle, "SPEECH_FAILED: unable to repeat the follow instruction.");
        return;
      }
      navigation_goal = send_navigation_goal(goal->destination, deadline, goal_handle);
      if (!navigation_goal) {
        say("I am not able to guide you to " + goal->destination +
            ". Please provide another location.");
        finish_failure(goal_handle, "NAVIGATION_GOAL_REJECTED: retry destination was not accepted.");
        return;
      }
    }
  }

  // Section: Internal navigation outcome keeps rejected, failed, canceled, and arrived distinct.
  enum class NavigationOutcome { ARRIVED, FAILED, CANCELED };

  // Special function: Submit one named Nav2 goal and return only an accepted handle.
  std::optional<rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr>
  send_navigation_goal(const std::string &destination,
                       const std::chrono::steady_clock::time_point &deadline,
                       const std::shared_ptr<GuideGoalHandle> &goal_handle) {
    if (!navigation_client_ || !navigation_client_->wait_for_action_server(
                                   std::chrono::duration<double>(goal_acceptance_timeout_sec_))) {
      return std::nullopt;
    }
    NavigateAction::Goal request;
    request.target_name = destination;
    auto future = navigation_client_->async_send_goal(request);
    if (!wait_future(future, deadline, goal_handle)) return std::nullopt;
    auto handle = future.get();
    if (!handle) return std::nullopt;
    return handle;
  }

  // Special function: Wait for the accepted Nav2 goal without treating acceptance as arrival.
  NavigationOutcome wait_navigation_result(
      const rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr &navigation_goal,
      const std::chrono::steady_clock::time_point &deadline,
      const std::shared_ptr<GuideGoalHandle> &goal_handle) {
    auto future = navigation_client_->async_get_result(navigation_goal);
    if (!wait_future(future, deadline, goal_handle)) {
      navigation_client_->async_cancel_goal(navigation_goal);
      return goal_handle->is_canceling() || !active_.load()
                 ? NavigationOutcome::CANCELED
                 : NavigationOutcome::FAILED;
    }
    const auto wrapped = future.get();
    return wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
                   wrapped.result->success
               ? NavigationOutcome::ARRIVED
               : NavigationOutcome::FAILED;
  }

  // Special function: Wait in small intervals so cancellation and the shared deadline remain responsive.
  template <typename FutureT>
  bool wait_future(FutureT &future, const std::chrono::steady_clock::time_point &deadline,
                   const std::shared_ptr<GuideGoalHandle> &goal_handle) {
    while (std::chrono::steady_clock::now() < deadline && active_.load() &&
           !goal_handle->is_canceling()) {
      if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready)
        return true;
    }
    return false;
  }

  // Special function: Send one bounded TTS request without granting it motion authority.
  bool say(const std::string &text) {
    if (!speech_client_ || !speech_client_->wait_for_service(
                               std::chrono::duration<double>(speech_timeout_sec_))) {
      return false;
    }
    auto request = std::make_shared<voice_interfaces::srv::SayText::Request>();
    request->text = text;
    auto future = speech_client_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(speech_timeout_sec_)) !=
        std::future_status::ready) {
      return false;
    }
    const auto response = future.get();
    return response && response->success;
  }

  // Special function: Publish the active sequential state to the action caller.
  void publish_feedback(const std::shared_ptr<GuideGoalHandle> &goal_handle,
                        const std::string &state) {
    auto feedback = std::make_shared<GuideAction::Feedback>();
    feedback->status = state;
    goal_handle->publish_feedback(feedback);
  }

  // Special function: Finish S3 with a successful guide result.
  void finish_success(const std::shared_ptr<GuideGoalHandle> &goal_handle,
                      const std::string &message) {
    auto result = std::make_shared<GuideAction::Result>();
    result->success = true;
    result->message = message;
    goal_handle->succeed(result);
    busy_.store(false);
  }

  // Special function: Finish a non-canceled state failure with its explicit status message.
  void finish_failure(const std::shared_ptr<GuideGoalHandle> &goal_handle,
                      const std::string &message) {
    auto result = std::make_shared<GuideAction::Result>();
    result->message = message;
    goal_handle->abort(result);
    busy_.store(false);
  }

  // Special function: Finish cancellation without incorrectly reporting person loss.
  void finish_canceled(const std::shared_ptr<GuideGoalHandle> &goal_handle) {
    auto result = std::make_shared<GuideAction::Result>();
    result->message = "GUIDE_PERSON canceled.";
    goal_handle->canceled(result);
    busy_.store(false);
  }

  // Special function: Drop lifecycle-owned handles after a completed guide mission.
  void release_resources() {
    action_server_.reset();
    navigation_client_.reset();
    speech_client_.reset();
    busy_.store(false);
  }

  std::atomic_bool active_{false};
  std::atomic_bool busy_{false};
  double default_timeout_sec_{120.0};
  double goal_acceptance_timeout_sec_{5.0};
  double speech_timeout_sec_{5.0};
  std::uint32_t max_navigation_retries_{1};
  rclcpp_action::Client<NavigateAction>::SharedPtr navigation_client_;
  rclcpp::Client<voice_interfaces::srv::SayText>::SharedPtr speech_client_;
  rclcpp_action::Server<GuideAction>::SharedPtr action_server_;
};

}  // namespace robot_yasmin_tasks

// Section: Run the guide action server with enough callback threads for Nav2 and TTS futures.
int main(int argc, char **argv) {
  // Section: The multi-threaded executor keeps lifecycle, Nav2, and service callbacks live.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_yasmin_tasks::GuidePersonServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
