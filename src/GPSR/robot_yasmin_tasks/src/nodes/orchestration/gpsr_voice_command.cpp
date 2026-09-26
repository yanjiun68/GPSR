// Section: Voice-command orchestration between TTS, STT, scheduler, and clarification
// topics.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iomanip>
#include <memory>
#include <mutex>
#include <random>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <voice_interfaces/srv/say_text.hpp>
#include <voice_interfaces/srv/transcribe_audio.hpp>

#include "robot_yasmin_tasks/runtime.hpp"

namespace robot_yasmin_tasks {

// Section: Voice intake, command correlation, scheduler-state gating, and clarification
// handling.
class GpsrVoiceCommandNode final : public rclcpp::Node {
 public:
  // Special function: Create unchanged voice, scheduler, and command interfaces.
  GpsrVoiceCommandNode() : rclcpp::Node("gpsr_voice_command") {
    const bool auto_start = declare_parameter<bool>("auto_start", true);
    declare_parameter<bool>("repeat", false);
    command_topic_ = declare_parameter<std::string>("command_topic", "/gpsr/command");
    scheduler_topic_ =
        declare_parameter<std::string>("scheduler_state_topic", "/gpsr/state");
    result_topic_ =
        declare_parameter<std::string>("command_result_topic", "/gpsr/command_result");
    clarification_topic_ = declare_parameter<std::string>(
        "clarification_response_topic", "/gpsr/clarification_response");
    declare_parameter<double>("record_sec", 6.0);
    declare_parameter<double>("listen_timeout_sec", 12.0);
    declare_parameter<double>("clarification_listen_timeout_sec", 20.0);
    declare_parameter<std::string>("prompt_text",
                                   "Please give me a GPSR command after the beep.");
    tts_ = create_client<voice_interfaces::srv::SayText>("/voice/tts/say");
    stt_service_ =
        create_client<voice_interfaces::srv::TranscribeAudio>("/voice/stt/transcribe");
    stt_pub_ = create_publisher<std_msgs::msg::Float32>("/voice/stt/start", 10);
    stt_sub_ = create_subscription<std_msgs::msg::String>(
        "/voice/stt/transcript", 10,
        std::bind(&GpsrVoiceCommandNode::stt_callback, this, std::placeholders::_1));
    command_pub_ = create_publisher<std_msgs::msg::String>(command_topic_, 10);
    clarification_pub_ =
        create_publisher<std_msgs::msg::String>(clarification_topic_, 10);
    scheduler_sub_ = create_subscription<std_msgs::msg::String>(
        scheduler_topic_, 10,
        std::bind(&GpsrVoiceCommandNode::scheduler_callback, this,
                  std::placeholders::_1));
    result_sub_ = create_subscription<std_msgs::msg::String>(
        result_topic_, 10,
        std::bind(&GpsrVoiceCommandNode::result_callback, this, std::placeholders::_1));
    if (auto_start) {
      start_timer_ = create_wall_timer(
          std::chrono::seconds(2), std::bind(&GpsrVoiceCommandNode::start_once, this));
    }
  }

 private:
  // Special function: Generate the same voice-owned correlation-ID shape.
  static std::string command_id() {
    std::random_device source;
    std::mt19937_64 generator(source());
    std::uniform_int_distribution<std::uint64_t> distribution;
    std::ostringstream stream;
    stream << "voice_" << std::hex << std::setfill('0') << std::setw(16)
           << distribution(generator) << std::setw(16) << distribution(generator);
    return stream.str();
  }

  // Special function: Wait for a ROS future while executor threads remain responsible
  // for spinning.
  template <typename Future>
  static bool wait_future(Future &future, double timeout) {
    return future.wait_for(std::chrono::duration<double>(timeout)) ==
           std::future_status::ready;
  }

  // Special function: Start one command worker only in scheduler-ready state.
  void start_once() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (busy_ || awaiting_result_ || scheduler_state_ != "WAITING_FOR_COMMAND") return;
    busy_ = true;
    std::thread([this]() { run_cycle(); }).detach();
  }

  // Special function: Prompt, listen, and submit exactly one correlated transcript.
  void run_cycle() {
    say(get_parameter("prompt_text").as_string());
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (scheduler_state_ != "WAITING_FOR_COMMAND") break;
      }
      const double timeout = get_parameter("listen_timeout_sec").as_double();
      RCLCPP_INFO(get_logger(), "Waiting for GPSR command (timeout: %.1f s).", timeout);
      const auto transcript = transcribe_once(timeout);
      if (!transcript.empty()) {
        std_msgs::msg::String command;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          active_command_id_ = command_id();
          awaiting_result_ = true;
          command.data = Json{
              {"type", "gpsr_voice_command"},
              {"command_id", active_command_id_},
              {"instruction",
               transcript}}.dump();
        }
        RCLCPP_INFO(get_logger(),
                    "GPSR transcript received; awaiting scheduler result.");
        command_pub_->publish(command);
        return;
      }
      RCLCPP_INFO(get_logger(),
                  "No command received within %.1f s; re-arming command input.",
                  timeout);
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!awaiting_result_) busy_ = false;
  }

  // Special function: Track scheduler readiness without interpreting other payload
  // fields.
  void scheduler_callback(const std_msgs::msg::String::SharedPtr message) {
    const auto payload = parse_json(message ? message->data : "", Json());
    if (!payload.is_object()) return;
    const auto state = payload.value("state", "");
    if (state.empty()) return;
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state == "WAITING_FOR_COMMAND" && scheduler_state_ != state) {
      RCLCPP_INFO(get_logger(),
                  "GPSR ready to receive command at current robot position.");
    }
    scheduler_state_ = state;
  }

  // Special function: Preserve the legacy topic transcript window.
  void stt_callback(const std_msgs::msg::String::SharedPtr message) {
    std::lock_guard<std::mutex> lock(stt_mutex_);
    if (!listening_) return;
    latest_ = message ? message->data : "";
    stt_condition_.notify_all();
  }

  // Special function: Correlate scheduler verdicts and clarification requests.
  void result_callback(const std_msgs::msg::String::SharedPtr message) {
    const auto result = parse_json(message ? message->data : "", Json());
    if (!result.is_object()) return;
    std::string active_id;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!awaiting_result_) return;
      active_id = active_command_id_;
    }
    const auto result_id = result.value("command_id", "");
    if (!active_id.empty() && result_id != active_id) return;
    if (result.contains("clarification") && result.at("clarification").is_object()) {
      const auto clarification = result.at("clarification");
      const int attempt = clarification.value("attempt", 0);
      const auto id = clarification.value("command_id", result_id);
      const auto prompt = clarification.value("prompt", "");
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (id.empty() || prompt.empty() || attempt <= clarification_attempt_) return;
        active_command_id_ = id;
        clarification_attempt_ = attempt;
      }
      std::thread([this, id, attempt, prompt]() {
        run_clarification(id, attempt, prompt);
      }).detach();
      return;
    }
    std::string response_text;
    // Section: A declined execution confirmation rejects only this command and reopens voice intake.
    if (result.value("request_new_command", false)) {
      response_text =
          "I will not execute that command. Please provide another GPSR command.";
    } else if (result.contains("execution") && result.at("execution").is_object()) {
      response_text = result.at("execution").value("success", false)
                          ? "I received the command."
                          : "The command is rejected.";
    } else if (!result.value("accepted", false)) {
      response_text = "The command is rejected.";
    }
    if (!response_text.empty()) say(response_text);
    std::lock_guard<std::mutex> lock(state_mutex_);
    awaiting_result_ = false;
    busy_ = false;
    active_command_id_.clear();
    clarification_attempt_ = 0;
  }

  // Special function: Speak and transcribe one clarification without guessing a slot.
  void run_clarification(const std::string &id, int attempt,
                         const std::string &prompt) {
    const auto transcript =
        say(prompt)
            ? transcribe_once(std::max(
                  1.0, get_parameter("clarification_listen_timeout_sec").as_double()))
            : "";
    std_msgs::msg::String reply;
    reply.data =
        Json{{"command_id", id}, {"attempt", attempt}, {"transcript", transcript}}
            .dump();
    clarification_pub_->publish(reply);
  }

  // Special function: Use request-scoped STT and return only a successful transcript.
  std::string transcribe_once(double timeout) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(std::max(0.0, timeout));
    const double discovery =
        std::min(2.0, std::max(0.0, std::chrono::duration<double>(
                                        deadline - std::chrono::steady_clock::now())
                                        .count()));
    if (!stt_service_->wait_for_service(std::chrono::duration<double>(discovery))) {
      return "";
    }
    auto request = std::make_shared<voice_interfaces::srv::TranscribeAudio::Request>();
    request->record_sec = static_cast<float>(get_parameter("record_sec").as_double());
    auto future = stt_service_->async_send_request(request);
    const double remaining =
        std::chrono::duration<double>(deadline - std::chrono::steady_clock::now())
            .count();
    if (remaining <= 0.0 || !wait_future(future, remaining)) return "";
    try {
      const auto response = future.get();
      return response && response->success ? response->transcript : "";
    } catch (const std::exception &) {
      return "";
    }
  }

  // Special function: Speak one prompt/status through the existing TTS service.
  bool say(const std::string &text) {
    if (!tts_->wait_for_service(std::chrono::seconds(3))) return false;
    auto request = std::make_shared<voice_interfaces::srv::SayText::Request>();
    request->text = text;
    auto future = tts_->async_send_request(request);
    return wait_future(future, 8.0);
  }

  std::mutex state_mutex_;
  std::mutex stt_mutex_;
  std::condition_variable stt_condition_;
  std::string command_topic_;
  std::string scheduler_topic_;
  std::string result_topic_;
  std::string clarification_topic_;
  std::string scheduler_state_{"INITIALIZING"};
  std::string latest_;
  bool busy_{false};
  bool awaiting_result_{false};
  bool listening_{false};
  std::string active_command_id_;
  int clarification_attempt_{0};
  rclcpp::Client<voice_interfaces::srv::SayText>::SharedPtr tts_;
  rclcpp::Client<voice_interfaces::srv::TranscribeAudio>::SharedPtr stt_service_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr stt_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stt_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr clarification_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scheduler_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;
  rclcpp::TimerBase::SharedPtr start_timer_;
};

}  // namespace robot_yasmin_tasks

// Section: Process startup and concurrent voice callback execution.
int main(int argc, char **argv) {
  // Section: Preserve the four-thread executor used by voice service waits and
  // callbacks.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_yasmin_tasks::GpsrVoiceCommandNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
