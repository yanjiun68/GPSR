#include <ament_index_cpp/get_package_prefix.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>

#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sm_receptionist/task_receptionist_process_utils.hpp"

namespace sm_receptionist::task_receptionist
{
// Section: lifecycle wrapper that keeps the Task Receptionist SMACC2 child dormant until activated.
class TaskReceptionistLifecycleNode : public nav2_util::LifecycleNode
{
public:
  using CallbackReturn = nav2_util::CallbackReturn;

  // Special function: declare every parameter forwarded into the detached SMACC2 child.
  TaskReceptionistLifecycleNode()
  : nav2_util::LifecycleNode("task_receptionist_lifecycle")
  {
    declare_parameter<bool>("dry_run", true);
    declare_parameter<int>("max_cycles", 2);
    declare_parameter<double>("wait_before_guest_sec", 3.0);
    declare_parameter<double>("post_feature_wait_sec", 2.0);
    declare_parameter<double>("startup_readiness_timeout_sec", 0.0);
    declare_parameter<std::string>("navigation_action_name", "/nav/navigate_named_pose");
    declare_parameter<std::string>("navigate_to_pose_action", "/navigate_to_pose");
    declare_parameter<std::string>("waypoint_7", "7");
    declare_parameter<std::string>("waypoint_6", "6");
    declare_parameter<std::string>("waypoint_8", "8");
    declare_parameter<std::string>("waypoint_99", "99");
    declare_parameter<std::string>("tts_action_name", "/tts/speak");
    declare_parameter<std::string>("dialogue_action_name", "/dialogue/collect_guest_profile");
    declare_parameter<std::string>("dialogue_mic_device", "auto");
    declare_parameter<double>("dialogue_record_sec", 4.0);
    declare_parameter<std::string>("dialogue_piper_model_path", "");
    declare_parameter<std::string>("dialogue_whisper_model", "base");
    declare_parameter<std::string>("dialogue_whisper_language", "en");
    declare_parameter<bool>("allow_dialogue_fallback", false);
    declare_parameter<bool>("store_profile_in_memory", true);
    declare_parameter<bool>("personal_test_mode", true);
    declare_parameter<std::string>("camera_index", "0");
    declare_parameter<std::string>("camera_backend", "realsense");
    declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
    declare_parameter<bool>("enable_depth", true);
    declare_parameter<std::string>("face_topic", "/perception/largest_face");
    declare_parameter<std::string>("appearance_topic", "/appearance/current_estimate");
    declare_parameter<double>("face_wait_timeout_sec", 10.0);
    declare_parameter<double>("appearance_wait_timeout_sec", 10.0);
    declare_parameter<double>("collection_duration_sec", 2.0);
    declare_parameter<int>("min_stable_samples", 3);
    declare_parameter<bool>("show_appearance_gui", true);
    declare_parameter<double>("stream_info_period_sec", 1.0);
    declare_parameter<bool>("force_process_exit_after_terminal_state", true);

    reap_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        reapFinishedSmaccProcess();
      });
  }

  // Special function: stop child processes if the lifecycle wrapper is destroyed.
  ~TaskReceptionistLifecycleNode() override
  {
    stopAllTrackedProcesses();
  }

protected:
  // Special function: cache configured parameters and clear stale process registries.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    dry_run_ = get_parameter("dry_run").as_bool();
    max_cycles_ = static_cast<int>(get_parameter("max_cycles").as_int());
    wait_before_guest_sec_ = get_parameter("wait_before_guest_sec").as_double();
    post_feature_wait_sec_ = get_parameter("post_feature_wait_sec").as_double();
    startup_readiness_timeout_sec_ = get_parameter("startup_readiness_timeout_sec").as_double();
    navigation_action_name_ = get_parameter("navigation_action_name").as_string();
    navigate_to_pose_action_ = get_parameter("navigate_to_pose_action").as_string();
    waypoint_7_ = get_parameter("waypoint_7").as_string();
    waypoint_6_ = get_parameter("waypoint_6").as_string();
    waypoint_8_ = get_parameter("waypoint_8").as_string();
    waypoint_99_ = get_parameter("waypoint_99").as_string();
    tts_action_name_ = get_parameter("tts_action_name").as_string();
    dialogue_action_name_ = get_parameter("dialogue_action_name").as_string();
    dialogue_mic_device_ = get_parameter("dialogue_mic_device").as_string();
    dialogue_record_sec_ = get_parameter("dialogue_record_sec").as_double();
    dialogue_piper_model_path_ = get_parameter("dialogue_piper_model_path").as_string();
    dialogue_whisper_model_ = get_parameter("dialogue_whisper_model").as_string();
    dialogue_whisper_language_ = get_parameter("dialogue_whisper_language").as_string();
    allow_dialogue_fallback_ = get_parameter("allow_dialogue_fallback").as_bool();
    store_profile_in_memory_ = get_parameter("store_profile_in_memory").as_bool();
    personal_test_mode_ = get_parameter("personal_test_mode").as_bool();
    camera_index_ = get_parameter("camera_index").as_string();
    camera_backend_ = get_parameter("camera_backend").as_string();
    image_topic_ = get_parameter("image_topic").as_string();
    enable_depth_ = get_parameter("enable_depth").as_bool();
    face_topic_ = get_parameter("face_topic").as_string();
    appearance_topic_ = get_parameter("appearance_topic").as_string();
    face_wait_timeout_sec_ = get_parameter("face_wait_timeout_sec").as_double();
    appearance_wait_timeout_sec_ = get_parameter("appearance_wait_timeout_sec").as_double();
    collection_duration_sec_ = get_parameter("collection_duration_sec").as_double();
    min_stable_samples_ = static_cast<int>(get_parameter("min_stable_samples").as_int());
    show_appearance_gui_ = get_parameter("show_appearance_gui").as_bool();
    stream_info_period_sec_ = get_parameter("stream_info_period_sec").as_double();
    force_process_exit_after_terminal_state_ =
      get_parameter("force_process_exit_after_terminal_state").as_bool();

    process_utils::clearInsightFacePgid();
    RCLCPP_INFO(
      get_logger(),
      "Configured Task Receptionist lifecycle: dry_run=%s max_cycles=%d waypoints=[%s,%s,%s,%s]",
      dry_run_ ? "true" : "false",
      max_cycles_,
      waypoint_7_.c_str(),
      waypoint_6_.c_str(),
      waypoint_8_.c_str(),
      waypoint_99_.c_str());
    return CallbackReturn::SUCCESS;
  }

  // Special function: start the detached SMACC2 child process when the lifecycle activates.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (smacc_pgid_ > 0 && process_utils::processGroupRunning(smacc_pgid_)) {
      RCLCPP_WARN(
        get_logger(),
        "Task Receptionist SMACC2 process group %d is already running.",
        static_cast<int>(smacc_pgid_));
      return CallbackReturn::SUCCESS;
    }

    const auto command = buildSmaccCommand();
    const auto pid = process_utils::launchDetachedProcess(command);
    if (pid <= 0) {
      RCLCPP_ERROR(get_logger(), "Failed to launch Task Receptionist SMACC2 process.");
      return CallbackReturn::FAILURE;
    }

    smacc_pgid_ = pid;
    smacc_command_ = command;
    RCLCPP_INFO(
      get_logger(),
      "Started Task Receptionist SMACC2 process group %d. Command: %s",
      static_cast<int>(smacc_pgid_),
      smacc_command_.c_str());

    rclcpp::sleep_for(std::chrono::milliseconds(500));
    if (!process_utils::processGroupRunning(smacc_pgid_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Task Receptionist SMACC2 process group %d exited immediately after launch.",
        static_cast<int>(smacc_pgid_));
      reapFinishedSmaccProcess();
      smacc_pgid_ = -1;
      return CallbackReturn::FAILURE;
    }

    createBond();
    return CallbackReturn::SUCCESS;
  }

  // Special function: stop children when the lifecycle deactivates.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    stopAllTrackedProcesses();
    return CallbackReturn::SUCCESS;
  }

  // Special function: clear child state during lifecycle cleanup.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    stopAllTrackedProcesses();
    process_utils::clearInsightFacePgid();
    return CallbackReturn::SUCCESS;
  }

  // Special function: stop child state during lifecycle shutdown.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    stopAllTrackedProcesses();
    process_utils::clearInsightFacePgid();
    return CallbackReturn::SUCCESS;
  }

  // Special function: stop child state during lifecycle error handling.
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    stopAllTrackedProcesses();
    process_utils::clearInsightFacePgid();
    return CallbackReturn::SUCCESS;
  }

private:
  // Special helper: build a quoted string ROS parameter argument.
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

  // Special helper: build a quoted boolean ROS parameter argument.
  std::string boolArg(const std::string & name, bool value) const
  {
    return process_utils::shellQuote(name + ":=" + std::string(value ? "true" : "false"));
  }

  // Special helper: build a quoted numeric ROS parameter argument.
  std::string numericArg(const std::string & name, double value) const
  {
    std::ostringstream out;
    out << name << ":=" << std::fixed << std::setprecision(3) << value;
    return process_utils::shellQuote(out.str());
  }

  // Special helper: build a quoted integer ROS parameter argument.
  std::string integerArg(const std::string & name, int value) const
  {
    return process_utils::shellQuote(name + ":=" + std::to_string(value));
  }

  // Special helper: resolve and build the full detached SMACC2 child command.
  std::string buildSmaccCommand() const
  {
    const auto package_prefix = ament_index_cpp::get_package_prefix("sm_receptionist");
    const auto executable = package_prefix + "/lib/sm_receptionist/task_receptionist_smacc2_node";

    return process_utils::shellQuote(executable) +
           " --ros-args " +
           "-p " + boolArg("dry_run", dry_run_) + " " +
           "-p " + integerArg("max_cycles", max_cycles_) + " " +
           "-p " + numericArg("wait_before_guest_sec", wait_before_guest_sec_) + " " +
           "-p " + numericArg("post_feature_wait_sec", post_feature_wait_sec_) + " " +
           "-p " + numericArg("startup_readiness_timeout_sec", startup_readiness_timeout_sec_) + " " +
           "-p " + yamlStringArg("navigation_action_name", navigation_action_name_) + " " +
           "-p " + yamlStringArg("navigate_to_pose_action", navigate_to_pose_action_) + " " +
           "-p " + yamlStringArg("waypoint_7", waypoint_7_) + " " +
           "-p " + yamlStringArg("waypoint_6", waypoint_6_) + " " +
           "-p " + yamlStringArg("waypoint_8", waypoint_8_) + " " +
           "-p " + yamlStringArg("waypoint_99", waypoint_99_) + " " +
           "-p " + yamlStringArg("tts_action_name", tts_action_name_) + " " +
           "-p " + yamlStringArg("dialogue_action_name", dialogue_action_name_) + " " +
           "-p " + yamlStringArg("dialogue_mic_device", dialogue_mic_device_) + " " +
           "-p " + numericArg("dialogue_record_sec", dialogue_record_sec_) + " " +
           "-p " + yamlStringArg("dialogue_piper_model_path", dialogue_piper_model_path_) + " " +
           "-p " + yamlStringArg("dialogue_whisper_model", dialogue_whisper_model_) + " " +
           "-p " + yamlStringArg("dialogue_whisper_language", dialogue_whisper_language_) + " " +
           "-p " + boolArg("allow_dialogue_fallback", allow_dialogue_fallback_) + " " +
           "-p " + boolArg("store_profile_in_memory", store_profile_in_memory_) + " " +
           "-p " + boolArg("personal_test_mode", personal_test_mode_) + " " +
           "-p " + yamlStringArg("camera_index", camera_index_) + " " +
           "-p " + yamlStringArg("camera_backend", camera_backend_) + " " +
           "-p " + yamlStringArg("image_topic", image_topic_) + " " +
           "-p " + boolArg("enable_depth", enable_depth_) + " " +
           "-p " + yamlStringArg("face_topic", face_topic_) + " " +
           "-p " + yamlStringArg("appearance_topic", appearance_topic_) + " " +
           "-p " + numericArg("face_wait_timeout_sec", face_wait_timeout_sec_) + " " +
           "-p " + numericArg("appearance_wait_timeout_sec", appearance_wait_timeout_sec_) + " " +
           "-p " + numericArg("collection_duration_sec", collection_duration_sec_) + " " +
           "-p " + integerArg("min_stable_samples", min_stable_samples_) + " " +
           "-p " + boolArg("show_appearance_gui", show_appearance_gui_) + " " +
           "-p " + numericArg("stream_info_period_sec", stream_info_period_sec_) + " " +
           "-p " + boolArg(
             "force_process_exit_after_terminal_state",
             force_process_exit_after_terminal_state_);
  }

  // Special helper: reap a finished SMACC2 child and shut down the lifecycle wrapper.
  void reapFinishedSmaccProcess()
  {
    if (smacc_pgid_ <= 0) {
      return;
    }

    int status = 0;
    const auto ret = waitpid(smacc_pgid_, &status, WNOHANG);
    if (ret == smacc_pgid_) {
      RCLCPP_INFO(
        get_logger(),
        "Reaped Task Receptionist SMACC2 process pid=%d status=%d.",
        static_cast<int>(smacc_pgid_),
        status);
      smacc_pgid_ = -1;
      process_utils::stopRegisteredInsightFace(get_logger());
      int exit_code = 1;
      if (WIFEXITED(status)) {
        exit_code = WEXITSTATUS(status);
      } else if (WIFSIGNALED(status)) {
        exit_code = 128 + WTERMSIG(status);
      }
      std::fflush(nullptr);
      std::_Exit(exit_code);
    }
  }

  // Special helper: stop both the InsightFace child tree and SMACC2 child tree.
  void stopAllTrackedProcesses()
  {
    process_utils::stopRegisteredInsightFace(get_logger());

    if (smacc_pgid_ > 0) {
      process_utils::stopProcessGroupWithPkill(
        smacc_pgid_,
        "Task Receptionist SMACC2",
        get_logger());
      reapFinishedSmaccProcess();
      smacc_pgid_ = -1;
    }
  }

  // Section: cached parameters forwarded to the SMACC2 child.
  bool dry_run_{true};
  int max_cycles_{2};
  double wait_before_guest_sec_{3.0};
  double post_feature_wait_sec_{2.0};
  double startup_readiness_timeout_sec_{0.0};
  std::string navigation_action_name_{"/nav/navigate_named_pose"};
  std::string navigate_to_pose_action_{"/navigate_to_pose"};
  std::string waypoint_7_{"7"};
  std::string waypoint_6_{"6"};
  std::string waypoint_8_{"8"};
  std::string waypoint_99_{"99"};
  std::string tts_action_name_{"/tts/speak"};
  std::string dialogue_action_name_{"/dialogue/collect_guest_profile"};
  std::string dialogue_mic_device_{"auto"};
  double dialogue_record_sec_{4.0};
  std::string dialogue_piper_model_path_;
  std::string dialogue_whisper_model_{"base"};
  std::string dialogue_whisper_language_{"en"};
  bool allow_dialogue_fallback_{false};
  bool store_profile_in_memory_{true};
  bool personal_test_mode_{true};
  std::string camera_index_{"0"};
  std::string camera_backend_{"realsense"};
  std::string image_topic_{"/camera/camera/color/image_raw"};
  bool enable_depth_{true};
  std::string face_topic_{"/perception/largest_face"};
  std::string appearance_topic_{"/appearance/current_estimate"};
  double face_wait_timeout_sec_{10.0};
  double appearance_wait_timeout_sec_{10.0};
  double collection_duration_sec_{2.0};
  int min_stable_samples_{3};
  bool show_appearance_gui_{true};
  double stream_info_period_sec_{1.0};
  bool force_process_exit_after_terminal_state_{true};
  pid_t smacc_pgid_{-1};
  std::string smacc_command_;
  rclcpp::TimerBase::SharedPtr reap_timer_;
};
}  // namespace sm_receptionist::task_receptionist

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sm_receptionist::task_receptionist::TaskReceptionistLifecycleNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
