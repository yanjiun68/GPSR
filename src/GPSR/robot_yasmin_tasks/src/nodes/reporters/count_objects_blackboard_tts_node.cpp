#include <algorithm>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <voice_interfaces/srv/say_text.hpp>

#include "robot_yasmin_tasks/runtime.hpp"

namespace robot_yasmin_tasks {

class CountObjectsBlackboardTTSNode final : public rclcpp::Node {
 public:
  // Special function: Create unchanged reporting interfaces and blackboard defaults.
  CountObjectsBlackboardTTSNode()
      : rclcpp::Node("count_objects_blackboard_tts_node"),
        blackboard_(std::make_shared<Blackboard>()) {
    topic_ = declare_parameter<std::string>("object_quantities_topic", "/perception/object_quantities");
    tts_name_ = declare_parameter<std::string>("tts_service_name", "/voice/tts/say");
    declare_parameter<bool>("speak_confirmed_only", true);
    declare_parameter<bool>("speak_empty_scene", false);
    declare_parameter<bool>("speak_on_change_only", true);
    declare_parameter<double>("min_report_interval_sec", 5.0);
    declare_parameter<int>("max_items_per_report", 8);
    declare_parameter<std::string>("report_style", "combined");
    blackboard_->set<Json>(bb::OBJECT_QUANTITIES, Json::object());
    blackboard_->set<int>(bb::COUNT_RESULT, 0);
    blackboard_->set<std::string>(bb::REPORT_TEXT, "");
    blackboard_->set<std::string>(bb::COUNT_OBJECTS_RESULT_JSON, "{}");
    subscription_ = create_subscription<std_msgs::msg::String>(
        topic_, 10, std::bind(&CountObjectsBlackboardTTSNode::quantity_callback,
                              this, std::placeholders::_1));
    tts_client_ = create_client<voice_interfaces::srv::SayText>(tts_name_);
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&CountObjectsBlackboardTTSNode::tts_timer,
                                         this));
    RCLCPP_INFO(get_logger(),
                "Count object blackboard TTS node ready: topic=%s, tts=%s",
                topic_.c_str(), tts_name_.c_str());
  }

 private:
  // Special function: Normalize confirmed positive object quantities and queue their report.
  void quantity_callback(const std_msgs::msg::String::SharedPtr message) {
    const auto payload = parse_json(message ? message->data : "", Json());
    if (!payload.is_object()) {
      RCLCPP_WARN(get_logger(), "Ignored invalid object quantity JSON.");
      return;
    }
    if (get_parameter("speak_confirmed_only").as_bool() &&
        !payload.value("confirmed", false)) return;
    std::map<std::string, int> quantities;
    if (payload.contains("object_quantities") &&
        payload.at("object_quantities").is_object()) {
      for (auto item = payload.at("object_quantities").begin();
           item != payload.at("object_quantities").end(); ++item) {
        try {
          const int count = item.value().get<int>();
          if (!item.key().empty() && count > 0) quantities[item.key()] = count;
        } catch (const Json::exception &) {
          continue;
        }
      }
    }
    Json normalized = Json::object();
    int total = 0;
    for (const auto &[name, count] : quantities) {
      normalized[name] = count;
      total += count;
    }
    const auto lines = build_report_lines(quantities);
    std::string report;
    for (const auto &line : lines) {
      if (!report.empty()) report += " ";
      report += line;
    }
    blackboard_->set<Json>(bb::OBJECT_QUANTITIES, normalized);
    blackboard_->set<int>(bb::COUNT_RESULT, total);
    blackboard_->set<std::string>(bb::COUNT_OBJECTS_RESULT_JSON, dump_json(payload));
    blackboard_->set<std::string>(bb::REPORT_TEXT, report);
    const auto fingerprint = dump_json(normalized);
    if (!should_queue(quantities.empty(), fingerprint)) return;
    speech_queue_.clear();
    speech_queue_.insert(speech_queue_.end(), lines.begin(), lines.end());
    last_fingerprint_ = fingerprint;
    last_report_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Queued object count report: %s", report.c_str());
  }

  // Special function: Apply unchanged empty/change/throttle policy.
  bool should_queue(bool empty, const std::string &fingerprint) const {
    if (empty && !get_parameter("speak_empty_scene").as_bool()) return false;
    if (get_parameter("speak_on_change_only").as_bool() &&
        fingerprint == last_fingerprint_) return false;
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_report_).count();
    return elapsed >= get_parameter("min_report_interval_sec").as_double();
  }

  // Special function: Apply the original small-number speech vocabulary.
  static std::string number_to_words(int count) {
    static const std::vector<std::string> words{
        "zero", "one", "two", "three", "four", "five", "six", "seven",
        "eight", "nine", "ten"};
    return count >= 0 && count <= 10
               ? words.at(static_cast<std::size_t>(count))
               : std::to_string(count);
  }

  // Special function: Preserve the simple English label pluralization rules.
  static std::string pluralize(const std::string &name) {
    if (name.size() >= 2 &&
        (name.compare(name.size() - 2, 2, "ch") == 0 ||
         name.compare(name.size() - 2, 2, "sh") == 0)) return name + "es";
    if (!name.empty() && (name.back() == 's' || name.back() == 'x')) return name + "es";
    if (name.size() > 1 && name.back() == 'y' &&
        std::string("aeiou").find(name[name.size() - 2]) == std::string::npos) {
      return name.substr(0, name.size() - 1) + "ies";
    }
    return name + "s";
  }

  // Special function: Format one count and object label for speech.
  static std::string phrase(std::string name, int count) {
    std::replace(name.begin(), name.end(), '_', ' ');
    return number_to_words(count) + " " + (count == 1 ? name : pluralize(name));
  }

  // Special function: Build combined or per-type report lines with the same truncation order.
  std::vector<std::string> build_report_lines(
      const std::map<std::string, int> &quantities) const {
    if (quantities.empty()) return {"I do not see any confirmed objects right now."};
    std::vector<std::pair<std::string, int>> items(quantities.begin(), quantities.end());
    std::sort(items.begin(), items.end(), [](const auto &left, const auto &right) {
      return left.second != right.second ? left.second > right.second
                                         : left.first < right.first;
    });
    const auto limit = static_cast<std::size_t>(
        std::max<std::int64_t>(1, get_parameter("max_items_per_report").as_int()));
    const auto visible = std::min(limit, items.size());
    const auto hidden = items.size() - visible;
    std::vector<std::string> phrases;
    for (std::size_t index = 0; index < visible; ++index) {
      phrases.push_back(phrase(items[index].first, items[index].second));
    }
    if (get_parameter("report_style").as_string() == "per_type") {
      std::vector<std::string> lines;
      for (const auto &value : phrases) lines.push_back("I can see " + value + ".");
      if (hidden > 0) lines.push_back("There are also " + std::to_string(hidden) +
                                      " more object types in view.");
      return lines;
    }
    std::string joined;
    if (phrases.size() == 1) joined = phrases[0];
    else if (phrases.size() == 2) joined = phrases[0] + " and " + phrases[1];
    else {
      for (std::size_t index = 0; index < phrases.size(); ++index) {
        if (index > 0) joined += index + 1 == phrases.size() ? ", and " : ", ";
        joined += phrases[index];
      }
    }
    std::string summary = "I can see " + joined + ".";
    if (hidden > 0) summary += " I am skipping " + std::to_string(hidden) +
                               " smaller object types to keep the report short.";
    return {summary};
  }

  // Special function: Drain one queued TTS request at a time.
  void tts_timer() {
    if (active_future_.valid()) {
      if (active_future_.wait_for(std::chrono::seconds(0)) !=
          std::future_status::ready) return;
      try {
        const auto response = active_future_.get();
        if (response && response->success) {
          RCLCPP_INFO(get_logger(), "Spoken object report line: %s",
                      active_text_.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "TTS failed for object report line: %s",
                      response ? response->message.c_str() : "");
        }
      } catch (const std::exception &error) {
        RCLCPP_WARN(get_logger(), "TTS failed for object report line: %s",
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
    // Section: Store only the shared future and avoid Humble's deprecated implicit conversion.
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

int main(int argc, char **argv) {
  // Section: Run the reporter in the standard single-node spin loop.
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_yasmin_tasks::CountObjectsBlackboardTTSNode>());
  rclcpp::shutdown();
  return 0;
}
