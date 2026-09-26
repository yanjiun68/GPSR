// Section: People-count blackboard reporter that emits debounced spoken summaries.
#include <algorithm>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <voice_interfaces/srv/say_text.hpp>

#include "robot_yasmin_tasks/runtime.hpp"

namespace robot_yasmin_tasks {

// Section: People-count ingestion, blackboard storage, change detection, and TTS
// reporting.
class CountPeopleBlackboardTTSNode final : public rclcpp::Node {
 public:
  // Special function: Create unchanged topic, TTS, blackboard, and timer interfaces.
  CountPeopleBlackboardTTSNode()
      : rclcpp::Node("count_people_blackboard_tts_node"),
        blackboard_(std::make_shared<Blackboard>()) {
    topic_ = declare_parameter<std::string>("human_quantities_topic",
                                            "/perception/human_quantities");
    tts_name_ = declare_parameter<std::string>("tts_service_name", "/voice/tts/say");
    declare_parameter<bool>("speak_confirmed_only", true);
    declare_parameter<bool>("speak_empty_scene", false);
    declare_parameter<bool>("speak_on_change_only", true);
    declare_parameter<double>("min_report_interval_sec", 5.0);
    blackboard_->set<int>(bb::COUNT_RESULT, 0);
    blackboard_->set<std::string>(bb::REPORT_TEXT, "");
    blackboard_->set<std::string>(bb::COUNT_PEOPLE_RESULT_JSON, "{}");
    blackboard_->set<Json>(bb::DETECTED_PEOPLE, Json::array());
    subscription_ = create_subscription<std_msgs::msg::String>(
        topic_, 10,
        std::bind(&CountPeopleBlackboardTTSNode::quantity_callback, this,
                  std::placeholders::_1));
    tts_client_ = create_client<voice_interfaces::srv::SayText>(tts_name_);
    timer_ =
        create_wall_timer(std::chrono::milliseconds(100),
                          std::bind(&CountPeopleBlackboardTTSNode::tts_timer, this));
    RCLCPP_INFO(get_logger(),
                "Count people blackboard TTS node ready: topic=%s, tts=%s",
                topic_.c_str(), tts_name_.c_str());
  }

 private:
  // Special function: Parse confirmed quantities and update every established
  // blackboard key.
  void quantity_callback(const std_msgs::msg::String::SharedPtr message) {
    const auto payload = parse_json(message ? message->data : "", Json());
    if (!payload.is_object()) {
      RCLCPP_WARN(get_logger(), "Ignored invalid human quantity JSON.");
      return;
    }
    if (get_parameter("speak_confirmed_only").as_bool() &&
        !payload.value("confirmed", false)) {
      return;
    }
    int count = 0;
    try {
      if (payload.contains("human_quantities") &&
          payload.at("human_quantities").is_object() &&
          payload.at("human_quantities").contains("person")) {
        count = std::max(0, payload.at("human_quantities").at("person").get<int>());
      } else {
        count = std::max(0, payload.value("total_detected_count", 0));
      }
    } catch (const Json::exception &) {
      count = 0;
    }
    Json people = Json::array();
    for (int index = 0; index < count; ++index) {
      people.push_back(Json{{"entity_id", "person_" + std::to_string(index + 1)},
                            {"name", "person"}});
    }
    const auto report = report_text(count);
    blackboard_->set<int>(bb::COUNT_RESULT, count);
    blackboard_->set<std::string>(bb::COUNT_PEOPLE_RESULT_JSON, dump_json(payload));
    blackboard_->set<Json>(bb::DETECTED_PEOPLE, people);
    blackboard_->set<std::string>(bb::REPORT_TEXT, report);
    if (!should_queue(count)) {
      return;
    }
    speech_queue_.clear();
    speech_queue_.push_back(report);
    last_fingerprint_ = std::to_string(count);
    last_report_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Queued people count report: %s", report.c_str());
  }

  // Special function: Apply unchanged empty/change/throttle reporting policy.
  bool should_queue(int count) const {
    if (count <= 0 && !get_parameter("speak_empty_scene").as_bool()) return false;
    if (get_parameter("speak_on_change_only").as_bool() &&
        std::to_string(count) == last_fingerprint_)
      return false;
    const auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_report_)
            .count();
    return elapsed >= get_parameter("min_report_interval_sec").as_double();
  }

  // Special function: Build exactly the singular/plural spoken people report.
  static std::string report_text(int count) {
    static const std::vector<std::string> words{"zero",  "one",  "two", "three",
                                                "four",  "five", "six", "seven",
                                                "eight", "nine", "ten"};
    if (count <= 0) return "I do not see any confirmed people right now.";
    if (count == 1) return "I can see one person.";
    const auto number =
        count <= 10 ? words.at(static_cast<std::size_t>(count)) : std::to_string(count);
    return "I can see " + number + " people.";
  }

  // Special function: Drain queued text asynchronously without blocking subscription
  // callbacks.
  void tts_timer() {
    if (active_future_.valid()) {
      if (active_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        return;
      try {
        const auto response = active_future_.get();
        if (response && response->success) {
          RCLCPP_INFO(get_logger(), "Spoken people report line: %s",
                      active_text_.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "TTS failed for people report line: %s",
                      response ? response->message.c_str() : "");
        }
      } catch (const std::exception &error) {
        RCLCPP_WARN(get_logger(), "TTS failed for people report line: %s",
                    error.what());
      }
      active_future_ = {};
      active_text_.clear();
      return;
    }
    if (speech_queue_.empty() || !tts_client_->service_is_ready()) return;
    active_text_ = speech_queue_.front();
    speech_queue_.pop_front();
    auto request = std::make_shared<voice_interfaces::srv::SayText::Request>();
    request->text = active_text_;
    // Section: Store only the shared future and avoid Humble's deprecated implicit
    // conversion.
    active_future_ = tts_client_->async_send_request(request).future.share();
  }

  Blackboard::SharedPtr blackboard_;
  std::string topic_;
  std::string tts_name_;
  std::string last_fingerprint_;
  std::chrono::steady_clock::time_point last_report_{};
  std::deque<std::string> speech_queue_;
  std::string active_text_;
  rclcpp::Client<voice_interfaces::srv::SayText>::SharedFuture active_future_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Client<voice_interfaces::srv::SayText>::SharedPtr tts_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_yasmin_tasks

// Section: Process startup and people-report callback execution.
int main(int argc, char **argv) {
  // Section: Run the reporter in the standard single-node spin loop.
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_yasmin_tasks::CountPeopleBlackboardTTSNode>());
  rclcpp::shutdown();
  return 0;
}
