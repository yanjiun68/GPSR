#include <ament_index_cpp/get_package_prefix.hpp>

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>

#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "insightface_feature_summary_smacc2/process_utils.hpp"

namespace insightface_feature_summary_smacc2
{
// Section: lifecycle wrapper that supervises the detached YASMIN process.
class FeatureSummaryLifecycleNode : public nav2_util::LifecycleNode
{
public:
  using CallbackReturn = nav2_util::CallbackReturn;

  // Special function: declare all parameters that must be forwarded into the YASMIN child.
  FeatureSummaryLifecycleNode()
  : nav2_util::LifecycleNode("insightface_feature_summary_lifecycle")
  {
    declare_parameter<bool>("personal_test_mode", true);
    declare_parameter<std::string>("camera_index", "0");
    declare_parameter<std::string>("camera_backend", "realsense");
    declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
    declare_parameter<bool>("enable_depth", true);
    declare_parameter<std::string>("face_topic", "/perception/largest_face");
    declare_parameter<std::string>("appearance_topic", "/appearance/current_estimate");
    declare_parameter<std::string>("tts_service_name", "/voice/tts/say");
    declare_parameter<double>("face_wait_timeout_sec", 10.0);
    declare_parameter<double>("appearance_wait_timeout_sec", 10.0);
    declare_parameter<double>("collection_duration_sec", 2.0);
    declare_parameter<int>("min_stable_samples", 3);
    declare_parameter<bool>("show_appearance_gui", true);
    declare_parameter<double>("stream_info_period_sec", 1.0);

    reap_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        reapFinishedYasminProcess();
      });
  }

  // Special function: stop child processes if the lifecycle wrapper is destroyed.
  ~FeatureSummaryLifecycleNode() override
  {
    stopAllTrackedProcesses();
  }

protected:
  // Special function: copy configured launch parameters into member variables.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    personal_test_mode_ = get_parameter("personal_test_mode").as_bool();
    camera_index_ = get_parameter("camera_index").as_string();
    camera_backend_ = get_parameter("camera_backend").as_string();
    image_topic_ = get_parameter("image_topic").as_string();
    enable_depth_ = get_parameter("enable_depth").as_bool();
    face_topic_ = get_parameter("face_topic").as_string();
    appearance_topic_ = get_parameter("appearance_topic").as_string();
    tts_service_name_ = get_parameter("tts_service_name").as_string();
    face_wait_timeout_sec_ = get_parameter("face_wait_timeout_sec").as_double();
    appearance_wait_timeout_sec_ = get_parameter("appearance_wait_timeout_sec").as_double();
    collection_duration_sec_ = get_parameter("collection_duration_sec").as_double();
    min_stable_samples_ = get_parameter("min_stable_samples").as_int();
    show_appearance_gui_ = get_parameter("show_appearance_gui").as_bool();
    stream_info_period_sec_ = get_parameter("stream_info_period_sec").as_double();

    process_utils::clearInsightFacePgid();
    RCLCPP_INFO(
      get_logger(),
      "Configured feature summary lifecycle: personal_test_mode=%s camera_index=%s "
      "appearance_topic=%s tts_service=%s",
      personal_test_mode_ ? "true" : "false",
      camera_index_.c_str(),
      appearance_topic_.c_str(),
      tts_service_name_.c_str());
    return CallbackReturn::SUCCESS;
  }

  // Special function: start the detached YASMIN state machine process.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (yasmin_pgid_ > 0 && process_utils::processGroupRunning(yasmin_pgid_)) {
      RCLCPP_WARN(
        get_logger(),
        "YASMIN feature summary process group %d is already running.",
        static_cast<int>(yasmin_pgid_));
      return CallbackReturn::SUCCESS;
    }

    const auto command = buildYasminCommand();
    const auto pid = process_utils::launchDetachedProcess(command);
    if (pid <= 0) {
      RCLCPP_ERROR(get_logger(), "Failed to launch YASMIN feature summary process.");
      return CallbackReturn::FAILURE;
    }

    yasmin_pgid_ = pid;
    yasmin_command_ = command;
    RCLCPP_INFO(
      get_logger(),
      "Started YASMIN feature summary with process group %d. Command: %s",
      static_cast<int>(yasmin_pgid_),
      yasmin_command_.c_str());

    rclcpp::sleep_for(std::chrono::milliseconds(500));
    if (!process_utils::processGroupRunning(yasmin_pgid_)) {
      RCLCPP_ERROR(
        get_logger(),
        "YASMIN feature summary process group %d exited immediately after launch.",
        static_cast<int>(yasmin_pgid_));
      reapFinishedYasminProcess();
      yasmin_pgid_ = -1;
      return CallbackReturn::FAILURE;
    }

    createBond();
    return CallbackReturn::SUCCESS;
  }

  // Special function: stop children when the lifecycle node deactivates.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    stopAllTrackedProcesses();
    return CallbackReturn::SUCCESS;
  }

  // Special function: clear process state when the lifecycle node cleans up.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    stopAllTrackedProcesses();
    process_utils::clearInsightFacePgid();
    return CallbackReturn::SUCCESS;
  }

  // Special function: clear process state during normal launch shutdown.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    stopAllTrackedProcesses();
    process_utils::clearInsightFacePgid();
    return CallbackReturn::SUCCESS;
  }

  // Special function: clear process state if lifecycle error handling is invoked.
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    stopAllTrackedProcesses();
    process_utils::clearInsightFacePgid();
    return CallbackReturn::SUCCESS;
  }

private:
  // Special helper: build a string ROS parameter argument for the detached child command.
  std::string yamlStringArg(const std::string & name, const std::string & value) const
  {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char ch : value) {
      if (ch == '"' || ch == '\\') {
        escaped.push_back('\\');
      }
      escaped.push_back(ch);
    }
    return process_utils::shellQuote(name + ":=\"" + escaped + "\"");
  }

  // Special helper: build a boolean ROS parameter argument for the detached child command.
  std::string boolArg(const std::string & name, bool value) const
  {
    return process_utils::shellQuote(name + ":=" + std::string(value ? "true" : "false"));
  }

  // Special helper: build a floating-point ROS parameter argument with stable precision.
  std::string numericArg(const std::string & name, double value) const
  {
    std::ostringstream out;
    out << name << ":=" << std::fixed << std::setprecision(3) << value;
    return process_utils::shellQuote(out.str());
  }

  // Special helper: build an integer ROS parameter argument.
  std::string integerArg(const std::string & name, int value) const
  {
    return process_utils::shellQuote(name + ":=" + std::to_string(value));
  }

  // Special helper: resolve and build the full detached YASMIN child command.
  std::string buildYasminCommand() const
  {
    const auto package_prefix =
      ament_index_cpp::get_package_prefix("insightface_feature_summary_smacc2");
    const auto executable =
      package_prefix +
      "/lib/insightface_feature_summary_smacc2/feature_summary_yasmin_node";

    return process_utils::shellQuote(executable) +
           " --ros-args " +
           "-p " + boolArg("personal_test_mode", personal_test_mode_) + " " +
           "-p " + yamlStringArg("camera_index", camera_index_) + " " +
           "-p " + yamlStringArg("camera_backend", camera_backend_) + " " +
           "-p " + yamlStringArg("image_topic", image_topic_) + " " +
           "-p " + boolArg("enable_depth", enable_depth_) + " " +
           "-p " + yamlStringArg("face_topic", face_topic_) + " " +
           "-p " + yamlStringArg("appearance_topic", appearance_topic_) + " " +
           "-p " + yamlStringArg("tts_service_name", tts_service_name_) + " " +
           "-p " + numericArg("face_wait_timeout_sec", face_wait_timeout_sec_) + " " +
           "-p " + numericArg("appearance_wait_timeout_sec", appearance_wait_timeout_sec_) + " " +
           "-p " + numericArg("collection_duration_sec", collection_duration_sec_) + " " +
           "-p " + integerArg("min_stable_samples", min_stable_samples_) + " " +
           "-p " + boolArg("show_appearance_gui", show_appearance_gui_) + " " +
           "-p " + numericArg("stream_info_period_sec", stream_info_period_sec_);
  }

  // Special helper: reap a finished YASMIN child and shut the wrapper down.
  void reapFinishedYasminProcess()
  {
    if (yasmin_pgid_ <= 0) {
      return;
    }

    int status = 0;
    const auto ret = waitpid(yasmin_pgid_, &status, WNOHANG);
    if (ret == yasmin_pgid_) {
      RCLCPP_INFO(
        get_logger(),
        "Reaped finished YASMIN feature summary process pid=%d.",
        static_cast<int>(yasmin_pgid_));
      yasmin_pgid_ = -1;
      process_utils::stopRegisteredInsightFace(get_logger());
      RCLCPP_INFO(get_logger(), "YASMIN feature summary finished. Shutting down orchestrator.");
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    }
  }

  // Special helper: stop both the InsightFace child tree and YASMIN child tree.
  void stopAllTrackedProcesses()
  {
    process_utils::stopRegisteredInsightFace(get_logger());

    if (yasmin_pgid_ > 0) {
      process_utils::stopProcessGroupWithPkill(
        yasmin_pgid_,
        "YASMIN feature summary",
        get_logger());
      reapFinishedYasminProcess();
      yasmin_pgid_ = -1;
    }
  }

  // Section: cached parameters forwarded to the YASMIN child.
  bool personal_test_mode_{true};
  std::string camera_index_{"0"};
  std::string camera_backend_{"realsense"};
  std::string image_topic_{"/camera/camera/color/image_raw"};
  bool enable_depth_{true};
  std::string face_topic_{"/perception/largest_face"};
  std::string appearance_topic_{"/appearance/current_estimate"};
  std::string tts_service_name_{"/voice/tts/say"};
  double face_wait_timeout_sec_{10.0};
  double appearance_wait_timeout_sec_{10.0};
  double collection_duration_sec_{2.0};
  int min_stable_samples_{3};
  bool show_appearance_gui_{true};
  double stream_info_period_sec_{1.0};
  pid_t yasmin_pgid_{-1};
  std::string yasmin_command_;
  rclcpp::TimerBase::SharedPtr reap_timer_;
};
}  // namespace insightface_feature_summary_smacc2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<insightface_feature_summary_smacc2::FeatureSummaryLifecycleNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
