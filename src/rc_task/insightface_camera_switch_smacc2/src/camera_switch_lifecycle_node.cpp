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

#include "insightface_camera_switch_smacc2/process_utils.hpp"

namespace insightface_camera_switch_smacc2
{
class CameraSwitchLifecycleNode : public nav2_util::LifecycleNode
{
public:
  using CallbackReturn = nav2_util::CallbackReturn;

  CameraSwitchLifecycleNode()
  : nav2_util::LifecycleNode("insightface_camera_switch_lifecycle")
  {
    declare_parameter<std::string>("camera0_index", "0");
    declare_parameter<std::string>("camera2_index", "2");
    declare_parameter<std::string>("face_topic", "/perception/largest_face");
    declare_parameter<double>("stage1_delay_sec", 3.0);
    declare_parameter<double>("stage2_hold_sec", 10.0);
    declare_parameter<double>("face_wait_timeout_sec", 0.0);
    declare_parameter<double>("stream_info_period_sec", 1.0);

    reap_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        reapFinishedSmaccProcess();
      });
  }

  ~CameraSwitchLifecycleNode() override
  {
    stopAllTrackedProcesses();
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    camera0_index_ = get_parameter("camera0_index").as_string();
    camera2_index_ = get_parameter("camera2_index").as_string();
    face_topic_ = get_parameter("face_topic").as_string();
    stage1_delay_sec_ = get_parameter("stage1_delay_sec").as_double();
    stage2_hold_sec_ = get_parameter("stage2_hold_sec").as_double();
    face_wait_timeout_sec_ = get_parameter("face_wait_timeout_sec").as_double();
    stream_info_period_sec_ = get_parameter("stream_info_period_sec").as_double();

    process_utils::clearInsightFacePgid();
    RCLCPP_INFO(
      get_logger(),
      "Configured camera switch lifecycle: camera0_index=%s camera2_index=%s face_topic=%s",
      camera0_index_.c_str(),
      camera2_index_.c_str(),
      face_topic_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (smacc_pgid_ > 0 && process_utils::processGroupRunning(smacc_pgid_)) {
      RCLCPP_WARN(
        get_logger(),
        "SMACC2 camera switch process group %d is already running.",
        static_cast<int>(smacc_pgid_));
      return CallbackReturn::SUCCESS;
    }

    const auto command = buildSmaccCommand();
    const auto pid = process_utils::launchDetachedProcess(command);
    if (pid <= 0) {
      RCLCPP_ERROR(get_logger(), "Failed to launch SMACC2 camera switch process.");
      return CallbackReturn::FAILURE;
    }

    smacc_pgid_ = pid;
    smacc_command_ = command;
    RCLCPP_INFO(
      get_logger(),
      "Started SMACC2 camera switch with process group %d. Command: %s",
      static_cast<int>(smacc_pgid_),
      smacc_command_.c_str());

    rclcpp::sleep_for(std::chrono::milliseconds(500));
    if (!process_utils::processGroupRunning(smacc_pgid_)) {
      RCLCPP_ERROR(
        get_logger(),
        "SMACC2 camera switch process group %d exited immediately after launch.",
        static_cast<int>(smacc_pgid_));
      reapFinishedSmaccProcess();
      smacc_pgid_ = -1;
      return CallbackReturn::FAILURE;
    }

    createBond();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    stopAllTrackedProcesses();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    stopAllTrackedProcesses();
    process_utils::clearInsightFacePgid();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    stopAllTrackedProcesses();
    process_utils::clearInsightFacePgid();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    destroyBond();
    stopAllTrackedProcesses();
    process_utils::clearInsightFacePgid();
    return CallbackReturn::SUCCESS;
  }

private:
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

  std::string numericArg(const std::string & name, double value) const
  {
    std::ostringstream out;
    out << name << ":=" << std::fixed << std::setprecision(3) << value;
    return process_utils::shellQuote(out.str());
  }

  std::string buildSmaccCommand() const
  {
    const auto package_prefix =
      ament_index_cpp::get_package_prefix("insightface_camera_switch_smacc2");
    const auto executable =
      package_prefix +
      "/lib/insightface_camera_switch_smacc2/camera_switch_smacc2_node";

    return process_utils::shellQuote(executable) +
           " --ros-args " +
           "-p " + yamlStringArg("camera0_index", camera0_index_) + " " +
           "-p " + yamlStringArg("camera2_index", camera2_index_) + " " +
           "-p " + yamlStringArg("face_topic", face_topic_) + " " +
           "-p " + numericArg("stage1_delay_sec", stage1_delay_sec_) + " " +
           "-p " + numericArg("stage2_hold_sec", stage2_hold_sec_) + " " +
           "-p " + numericArg("face_wait_timeout_sec", face_wait_timeout_sec_) + " " +
           "-p " + numericArg("stream_info_period_sec", stream_info_period_sec_);
  }

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
        "Reaped finished SMACC2 camera switch process pid=%d.",
      static_cast<int>(smacc_pgid_));
      smacc_pgid_ = -1;
      process_utils::stopRegisteredInsightFace(get_logger());
      RCLCPP_INFO(get_logger(), "SMACC2 camera switch finished. Shutting down orchestrator.");
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    }
  }

  void stopAllTrackedProcesses()
  {
    process_utils::stopRegisteredInsightFace(get_logger());

    if (smacc_pgid_ > 0) {
      process_utils::stopProcessGroupWithPkill(
        smacc_pgid_,
        "SMACC2 camera switch",
        get_logger());
      reapFinishedSmaccProcess();
      smacc_pgid_ = -1;
    }
  }

  std::string camera0_index_{"0"};
  std::string camera2_index_{"2"};
  std::string face_topic_{"/perception/largest_face"};
  double stage1_delay_sec_{3.0};
  double stage2_hold_sec_{10.0};
  double face_wait_timeout_sec_{0.0};
  double stream_info_period_sec_{1.0};
  pid_t smacc_pgid_{-1};
  std::string smacc_command_;
  rclcpp::TimerBase::SharedPtr reap_timer_;
};
}  // namespace insightface_camera_switch_smacc2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<insightface_camera_switch_smacc2::CameraSwitchLifecycleNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
