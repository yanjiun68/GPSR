#include <boost/mpl/list.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>
#include <std_msgs/msg/string.hpp>

#include "insightface_camera_switch_smacc2/process_utils.hpp"

namespace insightface_camera_switch_smacc2
{
namespace orthogonals
{
struct OrTask : smacc2::Orthogonal<OrTask>
{
};
}  // namespace orthogonals

struct StLaunchCamera0;
struct StWaitFaceCamera0;
struct StDelayAfterCamera0Face;
struct StStopCamera0;
struct StLaunchCamera2;
struct StWaitFaceCamera2;
struct StHoldAfterCamera2Face;
struct StStopAllAndFinish;
struct StAborted;

struct SmInsightFaceCameraSwitch
: smacc2::SmaccStateMachineBase<SmInsightFaceCameraSwitch, StLaunchCamera0>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    auto node = getNode();
    node->declare_parameter<std::string>("camera0_index", "0");
    node->declare_parameter<std::string>("camera2_index", "2");
    node->declare_parameter<std::string>("face_topic", "/perception/largest_face");
    node->declare_parameter<double>("stage1_delay_sec", 3.0);
    node->declare_parameter<double>("stage2_hold_sec", 10.0);
    node->declare_parameter<double>("face_wait_timeout_sec", 0.0);
    node->declare_parameter<double>("stream_info_period_sec", 1.0);

    setGlobalSMData("camera0_index", node->get_parameter("camera0_index").as_string());
    setGlobalSMData("camera2_index", node->get_parameter("camera2_index").as_string());
    setGlobalSMData("face_topic", node->get_parameter("face_topic").as_string());
    setGlobalSMData("stage1_delay_sec", node->get_parameter("stage1_delay_sec").as_double());
    setGlobalSMData("stage2_hold_sec", node->get_parameter("stage2_hold_sec").as_double());
    setGlobalSMData(
      "face_wait_timeout_sec",
      node->get_parameter("face_wait_timeout_sec").as_double());
    setGlobalSMData(
      "stream_info_period_sec",
      node->get_parameter("stream_info_period_sec").as_double());
    setGlobalSMData("active_insightface_pgid", static_cast<std::int64_t>(-1));

    process_utils::clearInsightFacePgid();
    createOrthogonal<orthogonals::OrTask>();

    RCLCPP_INFO(
      getLogger(),
      "SMACC2 InsightFace camera switch initialized: camera0_index=%s, camera2_index=%s, "
      "face_topic=%s",
      node->get_parameter("camera0_index").as_string().c_str(),
      node->get_parameter("camera2_index").as_string().c_str(),
      node->get_parameter("face_topic").as_string().c_str());
  }
};

namespace client_behaviors
{
std::string formatDouble(double value)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << value;
  return out.str();
}

std::string buildInsightFaceLaunchCommand(const std::string & camera_index, double stream_period)
{
  return std::string("ros2 launch face_detection appearance_pipeline.launch.py ") +
         "personal_test_mode:=true " +
         "camera_index:=" + process_utils::shellQuote(camera_index) + " " +
         "show_appearance_gui:=true " +
         "show_stream_info_overlay:=true " +
         "log_video_stats:=true " +
         "stream_info_period_sec:=" + process_utils::shellQuote(formatDouble(stream_period));
}

class CbLaunchInsightFace : public smacc2::SmaccAsyncClientBehavior
{
public:
  explicit CbLaunchInsightFace(std::string camera_index_key)
  : camera_index_key_(std::move(camera_index_key))
  {
  }

  void onEntry() override
  {
    std::string camera_index;
    double stream_period = 1.0;
    getStateMachine()->getGlobalSMData(camera_index_key_, camera_index);
    getStateMachine()->getGlobalSMData("stream_info_period_sec", stream_period);

    process_utils::stopRegisteredInsightFace(getLogger());

    const auto command = buildInsightFaceLaunchCommand(camera_index, stream_period);
    const auto pid = process_utils::launchDetachedProcess(command);
    if (pid <= 0) {
      RCLCPP_ERROR(
        getLogger(),
        "Failed to launch InsightFace for %s=%s.",
        camera_index_key_.c_str(),
        camera_index.c_str());
      postFailureEvent();
      return;
    }

    process_utils::writeInsightFacePgid(pid);
    getStateMachine()->setGlobalSMData(
      "active_insightface_pgid",
      static_cast<std::int64_t>(pid));

    RCLCPP_INFO(
      getLogger(),
      "Started InsightFace camera_index=%s with process group %d. Command: %s",
      camera_index.c_str(),
      static_cast<int>(pid),
      command.c_str());

    rclcpp::sleep_for(std::chrono::milliseconds(700));
    if (!process_utils::processGroupRunning(pid)) {
      RCLCPP_ERROR(
        getLogger(),
        "InsightFace process group %d exited immediately after launch.",
        static_cast<int>(pid));
      process_utils::clearInsightFacePgid();
      getStateMachine()->setGlobalSMData("active_insightface_pgid", static_cast<std::int64_t>(-1));
      postFailureEvent();
      return;
    }

    postSuccessEvent();
  }

private:
  std::string camera_index_key_;
};

class CbWaitForFace : public smacc2::SmaccAsyncClientBehavior
{
public:
  void onEntry() override
  {
    std::string face_topic = "/perception/largest_face";
    double timeout_sec = 0.0;
    getStateMachine()->getGlobalSMData("face_topic", face_topic);
    getStateMachine()->getGlobalSMData("face_wait_timeout_sec", timeout_sec);

    auto node = getNode();
    std::atomic_bool face_detected{false};
    auto subscription = node->create_subscription<std_msgs::msg::String>(
      face_topic,
      10,
      [&face_detected, this](const std_msgs::msg::String::SharedPtr msg) {
        try {
          const auto payload = nlohmann::json::parse(msg->data);
          if (payload.value("detected", false)) {
            face_detected.store(true, std::memory_order_relaxed);
          }
        } catch (const std::exception & exc) {
          RCLCPP_WARN_THROTTLE(
            getLogger(),
            *getNode()->get_clock(),
            2000,
            "Ignoring malformed face JSON: %s",
            exc.what());
        }
      });

    RCLCPP_INFO(
      getLogger(),
      "Waiting for face detection on %s%s.",
      face_topic.c_str(),
      timeout_sec > 0.0 ? " with timeout" : " without timeout");

    const auto start = std::chrono::steady_clock::now();
    while (
      rclcpp::ok() && !isShutdownRequested() &&
      !face_detected.load(std::memory_order_relaxed))
    {
      std::int64_t pgid = -1;
      getStateMachine()->getGlobalSMData("active_insightface_pgid", pgid);
      if (pgid > 0 && !process_utils::processGroupRunning(static_cast<pid_t>(pgid))) {
        RCLCPP_ERROR(
          getLogger(),
          "InsightFace process group %ld exited before face detection.",
          static_cast<long>(pgid));
        postFailureEvent();
        return;
      }

      if (timeout_sec > 0.0) {
        const auto elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start).count();
        if (elapsed >= timeout_sec) {
          RCLCPP_ERROR(
            getLogger(),
            "Timed out after %.2f seconds waiting for face detection on %s.",
            timeout_sec,
            face_topic.c_str());
          postFailureEvent();
          return;
        }
      }

      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (!face_detected.load(std::memory_order_relaxed)) {
      postFailureEvent();
      return;
    }

    RCLCPP_INFO(getLogger(), "Face detected on %s.", face_topic.c_str());
    postSuccessEvent();
  }
};

class CbWaitDuration : public smacc2::SmaccAsyncClientBehavior
{
public:
  explicit CbWaitDuration(std::string duration_key)
  : duration_key_(std::move(duration_key))
  {
  }

  void onEntry() override
  {
    double duration_sec = 0.0;
    getStateMachine()->getGlobalSMData(duration_key_, duration_sec);
    duration_sec = std::max(0.0, duration_sec);

    RCLCPP_INFO(
      getLogger(),
      "Waiting %.2f seconds for %s.",
      duration_sec,
      duration_key_.c_str());

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !isShutdownRequested()) {
      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      if (elapsed >= duration_sec) {
        postSuccessEvent();
        return;
      }

      std::int64_t pgid = -1;
      getStateMachine()->getGlobalSMData("active_insightface_pgid", pgid);
      if (pgid > 0 && !process_utils::processGroupRunning(static_cast<pid_t>(pgid))) {
        RCLCPP_ERROR(
          getLogger(),
          "InsightFace process group %ld exited during %s.",
          static_cast<long>(pgid),
          duration_key_.c_str());
        postFailureEvent();
        return;
      }

      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    postFailureEvent();
  }

private:
  std::string duration_key_;
};

class CbStopInsightFace : public smacc2::SmaccAsyncClientBehavior
{
public:
  void onEntry() override
  {
    std::int64_t pgid = -1;
    getStateMachine()->getGlobalSMData("active_insightface_pgid", pgid);
    if (pgid <= 0) {
      pgid = static_cast<std::int64_t>(process_utils::readInsightFacePgid());
    }

    const bool stopped = process_utils::stopProcessGroupWithPkill(
      static_cast<pid_t>(pgid),
      "InsightFace",
      getLogger());
    if (stopped) {
      process_utils::clearInsightFacePgid();
      getStateMachine()->setGlobalSMData("active_insightface_pgid", static_cast<std::int64_t>(-1));
      postSuccessEvent();
    } else {
      postFailureEvent();
    }
  }
};
}  // namespace client_behaviors

void requestSmaccShutdown()
{
  std::thread(
    []() {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    })
    .detach();
}

struct StLaunchCamera0 : smacc2::SmaccState<StLaunchCamera0, SmInsightFaceCameraSwitch>
{
  using SmaccState::SmaccState;
  using LaunchSuccess = smacc2::EvCbSuccess<
    client_behaviors::CbLaunchInsightFace,
    orthogonals::OrTask>;
  using LaunchFailure = smacc2::EvCbFailure<
    client_behaviors::CbLaunchInsightFace,
    orthogonals::OrTask>;

  using reactions = boost::mpl::list<
    smacc2::Transition<LaunchSuccess, StWaitFaceCamera0, smacc2::SUCCESS>,
    smacc2::Transition<LaunchFailure, StAborted, smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbLaunchInsightFace>(
      "camera0_index");
  }
};

struct StWaitFaceCamera0 : smacc2::SmaccState<StWaitFaceCamera0, SmInsightFaceCameraSwitch>
{
  using SmaccState::SmaccState;
  using WaitSuccess = smacc2::EvCbSuccess<client_behaviors::CbWaitForFace, orthogonals::OrTask>;
  using WaitFailure = smacc2::EvCbFailure<client_behaviors::CbWaitForFace, orthogonals::OrTask>;

  using reactions = boost::mpl::list<
    smacc2::Transition<WaitSuccess, StDelayAfterCamera0Face, smacc2::SUCCESS>,
    smacc2::Transition<WaitFailure, StAborted, smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbWaitForFace>();
  }
};

struct StDelayAfterCamera0Face
: smacc2::SmaccState<StDelayAfterCamera0Face, SmInsightFaceCameraSwitch>
{
  using SmaccState::SmaccState;
  using WaitSuccess = smacc2::EvCbSuccess<client_behaviors::CbWaitDuration, orthogonals::OrTask>;
  using WaitFailure = smacc2::EvCbFailure<client_behaviors::CbWaitDuration, orthogonals::OrTask>;

  using reactions = boost::mpl::list<
    smacc2::Transition<WaitSuccess, StStopCamera0, smacc2::SUCCESS>,
    smacc2::Transition<WaitFailure, StAborted, smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbWaitDuration>(
      "stage1_delay_sec");
  }
};

struct StStopCamera0 : smacc2::SmaccState<StStopCamera0, SmInsightFaceCameraSwitch>
{
  using SmaccState::SmaccState;
  using StopSuccess = smacc2::EvCbSuccess<client_behaviors::CbStopInsightFace, orthogonals::OrTask>;
  using StopFailure = smacc2::EvCbFailure<client_behaviors::CbStopInsightFace, orthogonals::OrTask>;

  using reactions = boost::mpl::list<
    smacc2::Transition<StopSuccess, StLaunchCamera2, smacc2::SUCCESS>,
    smacc2::Transition<StopFailure, StAborted, smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbStopInsightFace>();
  }
};

struct StLaunchCamera2 : smacc2::SmaccState<StLaunchCamera2, SmInsightFaceCameraSwitch>
{
  using SmaccState::SmaccState;
  using LaunchSuccess = smacc2::EvCbSuccess<
    client_behaviors::CbLaunchInsightFace,
    orthogonals::OrTask>;
  using LaunchFailure = smacc2::EvCbFailure<
    client_behaviors::CbLaunchInsightFace,
    orthogonals::OrTask>;

  using reactions = boost::mpl::list<
    smacc2::Transition<LaunchSuccess, StWaitFaceCamera2, smacc2::SUCCESS>,
    smacc2::Transition<LaunchFailure, StAborted, smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbLaunchInsightFace>(
      "camera2_index");
  }
};

struct StWaitFaceCamera2 : smacc2::SmaccState<StWaitFaceCamera2, SmInsightFaceCameraSwitch>
{
  using SmaccState::SmaccState;
  using WaitSuccess = smacc2::EvCbSuccess<client_behaviors::CbWaitForFace, orthogonals::OrTask>;
  using WaitFailure = smacc2::EvCbFailure<client_behaviors::CbWaitForFace, orthogonals::OrTask>;

  using reactions = boost::mpl::list<
    smacc2::Transition<WaitSuccess, StHoldAfterCamera2Face, smacc2::SUCCESS>,
    smacc2::Transition<WaitFailure, StAborted, smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbWaitForFace>();
  }
};

struct StHoldAfterCamera2Face
: smacc2::SmaccState<StHoldAfterCamera2Face, SmInsightFaceCameraSwitch>
{
  using SmaccState::SmaccState;
  using WaitSuccess = smacc2::EvCbSuccess<client_behaviors::CbWaitDuration, orthogonals::OrTask>;
  using WaitFailure = smacc2::EvCbFailure<client_behaviors::CbWaitDuration, orthogonals::OrTask>;

  using reactions = boost::mpl::list<
    smacc2::Transition<WaitSuccess, StStopAllAndFinish, smacc2::SUCCESS>,
    smacc2::Transition<WaitFailure, StAborted, smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbWaitDuration>(
      "stage2_hold_sec");
  }
};

struct StStopAllAndFinish
: smacc2::SmaccState<StStopAllAndFinish, SmInsightFaceCameraSwitch>
{
  using SmaccState::SmaccState;

  static void staticConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "Stopping remaining InsightFace processes and finishing.");
    process_utils::stopRegisteredInsightFace(getLogger());
    getStateMachine().setGlobalSMData("active_insightface_pgid", static_cast<std::int64_t>(-1));
    requestSmaccShutdown();
  }
};

struct StAborted : smacc2::SmaccState<StAborted, SmInsightFaceCameraSwitch>
{
  using SmaccState::SmaccState;

  static void staticConfigure() {}

  void onEntry()
  {
    RCLCPP_ERROR(getLogger(), "Camera switch state machine aborted. Cleaning InsightFace.");
    process_utils::stopRegisteredInsightFace(getLogger());
    getStateMachine().setGlobalSMData("active_insightface_pgid", static_cast<std::int64_t>(-1));
    requestSmaccShutdown();
  }
};
}  // namespace insightface_camera_switch_smacc2

namespace
{
void forceStopInsightFaceOnShutdown()
{
  const auto logger = rclcpp::get_logger("camera_switch_smacc2_shutdown");
  insightface_camera_switch_smacc2::process_utils::stopRegisteredInsightFace(logger);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::on_shutdown(forceStopInsightFaceOnShutdown);
  smacc2::run<insightface_camera_switch_smacc2::SmInsightFaceCameraSwitch>();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
