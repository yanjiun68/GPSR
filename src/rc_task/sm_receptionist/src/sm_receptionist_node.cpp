#include <boost/mpl/list.hpp>
#include <boost/statechart/event.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <action_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <llm_interfaces/msg/receptionist_context.hpp>
#include <llm_interfaces/srv/get_receptionist_advice.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <receptionist_interfaces/action/capture_named_profile.hpp>
#include <receptionist_interfaces/action/collect_guest_profile.hpp>
#include <receptionist_interfaces/action/navigate_named_pose.hpp>
#include <receptionist_interfaces/action/track_guest_appearance.hpp>
#include <receptionist_interfaces/msg/guest_appearance.hpp>
#include <receptionist_interfaces/msg/person_profile.hpp>
#include <receptionist_interfaces/srv/clear_current_guest.hpp>
#include <receptionist_interfaces/srv/commit_current_guest.hpp>
#include <receptionist_interfaces/srv/face_pose.hpp>
#include <receptionist_interfaces/srv/find_empty_chair.hpp>
#include <receptionist_interfaces/srv/mark_profile_seated.hpp>
#include <receptionist_interfaces/srv/point_to_seat.hpp>
#include <receptionist_interfaces/srv/prepare_navigation_session.hpp>
#include <receptionist_interfaces/srv/resolve_presentation_targets.hpp>
#include <receptionist_interfaces/srv/set_current_guest_profile.hpp>
#include <receptionist_interfaces/srv/set_host_profile.hpp>
#include <receptionist_interfaces/srv/speak_text.hpp>
#include <smacc2/smacc.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

namespace sm_receptionist
{
namespace orthogonals
{
struct OrTask : smacc2::Orthogonal<OrTask>
{
};
}  // namespace orthogonals

namespace events
{
struct EvNextGuest : boost::statechart::event<EvNextGuest>
{
};

struct EvAllGuestsDone : boost::statechart::event<EvAllGuestsDone>
{
};
}  // namespace events

struct StStartupGate;
struct StInitHostMemory;
struct StWaitGuest;
struct StApproachGuest;
struct StGreetGuest;
struct StCollectProfile;
struct StTrackAppearance;
struct StStoreGuest;
struct StCommitGuest;
struct StGoLivingRoom;
struct StResolveTargets;
struct StFaceTarget;
struct StCaptureHostFace;
struct StIntroduceGuest;
struct StFindEmptyChair;
struct StFaceChair;
struct StPointChair;
struct StAskSit;
struct StMarkSeated;
struct StCheckLoop;
struct StReturnReception;
struct StReturnStart;
struct StFinished;
struct StAborted;

struct SmReceptionist : smacc2::SmaccStateMachineBase<SmReceptionist, StStartupGate>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    auto node = getNode();

    node->declare_parameter<bool>("dry_run", true);
    node->declare_parameter<int>("target_guest_count", 2);
    node->declare_parameter<std::string>("startup_dio13_topic", "/dio13_state");
    node->declare_parameter<std::string>("guest_enter_topic", "/person_selected");
    node->declare_parameter<double>("wait_guest_timeout_sec", 10.0);
    node->declare_parameter<double>("startup_timeout_sec", 0.0);
    node->declare_parameter<std::string>("waypoint_1", "ENTRANCE");
    node->declare_parameter<std::string>("waypoint_2", "FACE_HOST");
    node->declare_parameter<std::string>("waypoint_3", "FACE_CHAIR1");
    node->declare_parameter<std::string>("waypoint_4", "FACE_CHAIR3");
    node->declare_parameter<std::string>("waypoint_5", "INITIAL_POINT");
    node->declare_parameter<bool>("force_process_exit_after_terminal_state", true);

    setGlobalSMData("dry_run", node->get_parameter("dry_run").as_bool());
    setGlobalSMData("target_guest_count", static_cast<int>(node->get_parameter("target_guest_count").as_int()));
    setGlobalSMData("guest_index", 1);
    setGlobalSMData("committed_guest_count", 0);
    setGlobalSMData("current_guest_profile_id", std::string("current_guest"));
    setGlobalSMData("current_guest_committed_profile_id", std::string(""));
    setGlobalSMData("guest_name", std::string(""));
    setGlobalSMData("favorite_drink", std::string(""));
    setGlobalSMData("selected_chair_id", std::string(""));
    setGlobalSMData("selected_chair_waist_yaw_rad", 0.0);
    setGlobalSMData("target_profile_ids_csv", std::string("host"));
    setGlobalSMData("last_error", std::string(""));

    createOrthogonal<orthogonals::OrTask>();

    RCLCPP_INFO(
      getLogger(),
      "SMACC2 receptionist initialized: dry_run=%s target_guest_count=%ld",
      node->get_parameter("dry_run").as_bool() ? "true" : "false",
      node->get_parameter("target_guest_count").as_int());
  }
};

namespace client_behaviors
{
class CbReceptionistStep : public smacc2::SmaccAsyncClientBehavior
{
public:
  explicit CbReceptionistStep(std::string step_name)
  : step_name_(std::move(step_name))
  {
  }

  void onEntry() override
  {
    const bool ok = runStep();
    if (ok) {
      postSuccessEvent();
    } else {
      postFailureEvent();
    }
  }

private:
  bool runStep()
  {
    if (step_name_ == "startup_gate") {
      return startupGate();
    }
    if (step_name_ == "init_host_memory") {
      return initHostMemory();
    }
    if (step_name_ == "wait_guest") {
      return waitGuest();
    }
    if (step_name_ == "approach_guest") {
      return navigateToParam("waypoint_1", "ENTRANCE");
    }
    if (step_name_ == "greet_guest") {
      return speakWithAdvice("greeting", "Hi, welcome, nice to meet you.");
    }
    if (step_name_ == "collect_profile") {
      return collectProfile();
    }
    if (step_name_ == "track_appearance") {
      return trackAppearance();
    }
    if (step_name_ == "store_guest") {
      return storeGuest();
    }
    if (step_name_ == "commit_guest") {
      return commitGuest();
    }
    if (step_name_ == "go_living_room") {
      return navigateToParam("waypoint_2", "FACE_HOST");
    }
    if (step_name_ == "resolve_targets") {
      return resolveTargets();
    }
    if (step_name_ == "face_target") {
      return facePoseFromKey("face_target_pose");
    }
    if (step_name_ == "capture_host_face") {
      return captureHostFace();
    }
    if (step_name_ == "introduce_guest") {
      return introduceGuest();
    }
    if (step_name_ == "find_empty_chair") {
      return findEmptyChair();
    }
    if (step_name_ == "face_chair") {
      return facePoseFromKey("selected_chair_pose");
    }
    if (step_name_ == "point_chair") {
      return pointChair();
    }
    if (step_name_ == "ask_sit") {
      return askSit();
    }
    if (step_name_ == "mark_seated") {
      return markSeated();
    }
    if (step_name_ == "return_reception") {
      return navigateToParam("waypoint_1", "ENTRANCE");
    }
    if (step_name_ == "return_start") {
      return navigateToParam("waypoint_5", "INITIAL_POINT");
    }
    RCLCPP_ERROR(getLogger(), "Unknown receptionist step: %s", step_name_.c_str());
    return false;
  }

  bool dryRun()
  {
    bool dry_run = true;
    getStateMachine()->getGlobalSMData("dry_run", dry_run);
    return dry_run;
  }

  int guestIndex()
  {
    int guest_index = 1;
    getStateMachine()->getGlobalSMData("guest_index", guest_index);
    return guest_index;
  }

  std::string stringParam(const std::string & name, const std::string & fallback)
  {
    auto node = getNode();
    if (!node->has_parameter(name)) {
      node->declare_parameter<std::string>(name, fallback);
    }
    return node->get_parameter(name).as_string();
  }

  double doubleParam(const std::string & name, double fallback)
  {
    auto node = getNode();
    if (!node->has_parameter(name)) {
      node->declare_parameter<double>(name, fallback);
    }
    return node->get_parameter(name).as_double();
  }

  bool waitForBoolTopic(const std::string & topic_name, double timeout_sec, bool success_on_timeout)
  {
    if (dryRun()) {
      RCLCPP_INFO(getLogger(), "Dry run: skipping wait on %s.", topic_name.c_str());
      return true;
    }

    auto node = getNode();
    auto received_true = std::make_shared<std::atomic_bool>(false);
    auto sub = node->create_subscription<std_msgs::msg::Bool>(
      topic_name,
      10,
      [received_true](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          received_true->store(true, std::memory_order_relaxed);
        }
      });

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !received_true->load(std::memory_order_relaxed)) {
      if (timeout_sec > 0.0) {
        const auto elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start).count();
        if (elapsed >= timeout_sec) {
          return success_on_timeout;
        }
      }
      rclcpp::sleep_for(100ms);
    }

    return received_true->load(std::memory_order_relaxed);
  }

  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr callService(
    const std::string & service_name,
    const typename ServiceT::Request::SharedPtr & request,
    double wait_sec,
    double response_sec)
  {
    auto node = getNode();
    auto client = node->create_client<ServiceT>(service_name);
    if (!client->wait_for_service(std::chrono::duration<double>(wait_sec))) {
      RCLCPP_WARN(getLogger(), "Service unavailable: %s", service_name.c_str());
      return nullptr;
    }
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(response_sec)) != std::future_status::ready) {
      RCLCPP_WARN(getLogger(), "Service timed out: %s", service_name.c_str());
      return nullptr;
    }
    return future.get();
  }

  template<typename ActionT>
  typename ActionT::Result::SharedPtr callAction(
    const std::string & action_name,
    const typename ActionT::Goal & goal,
    double wait_sec,
    double result_sec)
  {
    auto node = getNode();
    auto client = rclcpp_action::create_client<ActionT>(node, action_name);
    if (!client->wait_for_action_server(std::chrono::duration<double>(wait_sec))) {
      RCLCPP_WARN(getLogger(), "Action unavailable: %s", action_name.c_str());
      return nullptr;
    }

    auto goal_future = client->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::duration<double>(wait_sec)) != std::future_status::ready) {
      RCLCPP_WARN(getLogger(), "Action goal timed out: %s", action_name.c_str());
      return nullptr;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_WARN(getLogger(), "Action goal rejected: %s", action_name.c_str());
      return nullptr;
    }

    auto result_future = client->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::duration<double>(result_sec)) != std::future_status::ready) {
      RCLCPP_WARN(getLogger(), "Action result timed out: %s", action_name.c_str());
      return nullptr;
    }

    auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(getLogger(), "Action ended without success: %s", action_name.c_str());
      return nullptr;
    }

    return wrapped_result.result;
  }

  bool startupGate()
  {
    const auto topic = stringParam("startup_dio13_topic", "/dio13_state");
    const auto timeout = doubleParam("startup_timeout_sec", 0.0);
    return waitForBoolTopic(topic, timeout, false);
  }

  bool initHostMemory()
  {
    if (dryRun()) {
      RCLCPP_INFO(getLogger(), "Dry run: initialized host memory.");
      return true;
    }

    auto host = receptionist_interfaces::msg::PersonProfile();
    host.profile_id = "host";
    host.role = "host";
    host.name = "Host";
    host.favorite_drink = "tea";
    host.gender = "unknown";
    host.is_confirmed = true;
    host.confidence_score = 1.0;

    auto set_host_req = std::make_shared<receptionist_interfaces::srv::SetHostProfile::Request>();
    set_host_req->profile = host;
    auto set_host_resp = callService<receptionist_interfaces::srv::SetHostProfile>(
      "/guest_memory/set_host_profile", set_host_req, 2.0, 2.0);

    auto clear_req = std::make_shared<receptionist_interfaces::srv::ClearCurrentGuest::Request>();
    auto clear_resp = callService<receptionist_interfaces::srv::ClearCurrentGuest>(
      "/guest_memory/clear_current_guest", clear_req, 2.0, 2.0);

    auto prepare_req =
      std::make_shared<receptionist_interfaces::srv::PrepareNavigationSession::Request>();
    auto prepare_resp = callService<receptionist_interfaces::srv::PrepareNavigationSession>(
      "/nav/prepare_session", prepare_req, 3.0, 5.0);

    return set_host_resp && set_host_resp->success && clear_resp && clear_resp->success &&
           prepare_resp && prepare_resp->success;
  }

  bool waitGuest()
  {
    const auto topic = stringParam("guest_enter_topic", "/person_selected");
    const auto timeout = doubleParam("wait_guest_timeout_sec", 10.0);
    return waitForBoolTopic(topic, timeout, true);
  }

  bool navigateToParam(const std::string & param_name, const std::string & fallback)
  {
    const auto target = stringParam(param_name, fallback);
    if (dryRun()) {
      RCLCPP_INFO(getLogger(), "Dry run: navigating to %s.", target.c_str());
      return true;
    }

    receptionist_interfaces::action::NavigateNamedPose::Goal goal;
    goal.target_name = target;
    auto result = callAction<receptionist_interfaces::action::NavigateNamedPose>(
      "/nav/navigate_named_pose", goal, 5.0, 180.0);
    return result && result->success;
  }

  bool speakText(const std::string & text)
  {
    if (dryRun()) {
      RCLCPP_INFO(getLogger(), "Dry run speech: %s", text.c_str());
      return true;
    }

    auto req = std::make_shared<receptionist_interfaces::srv::SpeakText::Request>();
    req->text = text;
    auto resp = callService<receptionist_interfaces::srv::SpeakText>(
      "/speech/speak_text", req, 3.0, 20.0);
    return resp && resp->success;
  }

  llm_interfaces::msg::ReceptionistContext buildLlmContext(const std::string & state)
  {
    std::string guest_name;
    std::string favorite_drink;
    std::string profile_id;
    std::string last_error;
    std::string target_ids_csv;
    int target_guest_count = 2;
    getStateMachine()->getGlobalSMData("guest_name", guest_name);
    getStateMachine()->getGlobalSMData("favorite_drink", favorite_drink);
    getStateMachine()->getGlobalSMData("current_guest_committed_profile_id", profile_id);
    getStateMachine()->getGlobalSMData("last_error", last_error);
    getStateMachine()->getGlobalSMData("target_profile_ids_csv", target_ids_csv);
    getStateMachine()->getGlobalSMData("target_guest_count", target_guest_count);

    llm_interfaces::msg::ReceptionistContext context;
    context.task_id = "receptionist";
    context.current_state = state;
    context.guest_index = guestIndex();
    context.target_guest_count = target_guest_count;
    context.profile_id = profile_id;
    context.guest_name = guest_name;
    context.favorite_drink = favorite_drink;
    context.target_profile_ids = splitCsv(target_ids_csv);
    context.last_error = last_error;
    context.dry_run = dryRun();
    return context;
  }

  std::vector<std::string> splitCsv(const std::string & csv_text)
  {
    std::vector<std::string> values;
    std::string current;
    for (const auto character : csv_text) {
      if (character == ',') {
        if (!current.empty()) {
          values.push_back(current);
        }
        current.clear();
      } else {
        current.push_back(character);
      }
    }
    if (!current.empty()) {
      values.push_back(current);
    }
    return values;
  }

  std::string joinCsv(const std::vector<std::string> & values)
  {
    std::string csv_text;
    for (const auto & value : values) {
      if (!csv_text.empty()) {
        csv_text += ",";
      }
      csv_text += value;
    }
    return csv_text;
  }

  std::string adviceSpeech(const std::string & request_type, const std::string & fallback)
  {
    auto req = std::make_shared<llm_interfaces::srv::GetReceptionistAdvice::Request>();
    req->request_type = request_type;
    req->context = buildLlmContext(step_name_);

    auto resp = callService<llm_interfaces::srv::GetReceptionistAdvice>(
      "/llm/receptionist_advice", req, 0.3, 2.0);
    if (resp && !resp->speech_text.empty()) {
      return resp->speech_text;
    }
    return fallback;
  }

  bool speakWithAdvice(const std::string & request_type, const std::string & fallback)
  {
    return speakText(adviceSpeech(request_type, fallback));
  }

  receptionist_interfaces::msg::PersonProfile fallbackProfile()
  {
    const int index = guestIndex();
    receptionist_interfaces::msg::PersonProfile profile;
    profile.profile_id = "current_guest";
    profile.role = "guest";
    profile.name = "Guest " + std::to_string(index);
    profile.favorite_drink = "water";
    profile.gender = "unknown";
    profile.is_confirmed = true;
    profile.confidence_score = 0.5;
    return profile;
  }

  bool collectProfile()
  {
    receptionist_interfaces::msg::PersonProfile profile = fallbackProfile();

    if (!dryRun()) {
      receptionist_interfaces::action::CollectGuestProfile::Goal goal;
      goal.mic_device = stringParam("dialogue_mic_device", "auto");
      goal.record_sec = static_cast<float>(doubleParam("dialogue_record_sec", 4.0));
      goal.piper_model_path = stringParam("dialogue_piper_model_path", "");
      goal.whisper_model = stringParam("dialogue_whisper_model", "base");
      goal.whisper_language = stringParam("dialogue_whisper_language", "en");

      auto result = callAction<receptionist_interfaces::action::CollectGuestProfile>(
        "/dialogue/collect_guest_profile", goal, 3.0, 60.0);
      if (result && result->success) {
        profile = result->profile;
      } else {
        RCLCPP_WARN(getLogger(), "Dialogue failed; continuing with fallback profile.");
      }
    }

    profile.profile_id = "current_guest";
    profile.role = "guest";
    getStateMachine()->setGlobalSMData("guest_name", profile.name);
    getStateMachine()->setGlobalSMData("favorite_drink", profile.favorite_drink);
    getStateMachine()->setGlobalSMData("current_guest_profile_id", profile.profile_id);
    return true;
  }

  bool trackAppearance()
  {
    if (dryRun()) {
      return true;
    }

    receptionist_interfaces::action::TrackGuestAppearance::Goal goal;
    goal.guest_profile_id = "current_guest";
    auto node = getNode();
    auto client = rclcpp_action::create_client<receptionist_interfaces::action::TrackGuestAppearance>(
      node, "/appearance/track_guest");
    if (!client->wait_for_action_server(2s)) {
      return true;
    }
    auto goal_future = client->async_send_goal(goal);
    if (goal_future.wait_for(5s) != std::future_status::ready) {
      return true;
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      return true;
    }
    rclcpp::sleep_for(1s);
    client->async_cancel_goal(goal_handle);
    return true;
  }

  bool storeGuest()
  {
    receptionist_interfaces::msg::PersonProfile profile = fallbackProfile();
    getStateMachine()->getGlobalSMData("guest_name", profile.name);
    getStateMachine()->getGlobalSMData("favorite_drink", profile.favorite_drink);

    if (dryRun()) {
      return true;
    }

    auto req = std::make_shared<receptionist_interfaces::srv::SetCurrentGuestProfile::Request>();
    req->profile = profile;
    auto resp = callService<receptionist_interfaces::srv::SetCurrentGuestProfile>(
      "/guest_memory/set_current_guest_profile", req, 2.0, 2.0);
    return resp && resp->success;
  }

  bool commitGuest()
  {
    const std::string profile_id = "guest_" + std::to_string(guestIndex());
    getStateMachine()->setGlobalSMData("current_guest_committed_profile_id", profile_id);

    if (dryRun()) {
      int count = 0;
      getStateMachine()->getGlobalSMData("committed_guest_count", count);
      getStateMachine()->setGlobalSMData("committed_guest_count", count + 1);
      return true;
    }

    auto req = std::make_shared<receptionist_interfaces::srv::CommitCurrentGuest::Request>();
    req->new_profile_id = profile_id;
    auto resp = callService<receptionist_interfaces::srv::CommitCurrentGuest>(
      "/guest_memory/commit_current_guest", req, 2.0, 2.0);
    if (resp && resp->success) {
      getStateMachine()->setGlobalSMData(
        "committed_guest_count", static_cast<int>(resp->confirmed_guest_count));
      return true;
    }
    return false;
  }

  bool resolveTargets()
  {
    std::vector<std::string> target_ids{"host"};
    if (!dryRun()) {
      auto req =
        std::make_shared<receptionist_interfaces::srv::ResolvePresentationTargets::Request>();
      req->guest_index = guestIndex();
      auto resp = callService<receptionist_interfaces::srv::ResolvePresentationTargets>(
        "/seated_people/resolve_presentation_targets", req, 3.0, 5.0);
      if (resp && resp->success) {
        target_ids = resp->target_profile_ids;
      }
    }

    getStateMachine()->setGlobalSMData("target_profile_ids_csv", joinCsv(target_ids));
    return true;
  }

  bool facePoseFromKey(const std::string & key)
  {
    if (dryRun()) {
      return true;
    }

    (void)key;
    geometry_msgs::msg::PoseStamped pose;
    auto req = std::make_shared<receptionist_interfaces::srv::FacePose::Request>();
    req->target_pose = pose;
    auto resp = callService<receptionist_interfaces::srv::FacePose>(
      "/nav/face_pose", req, 3.0, 20.0);
    return resp && resp->success;
  }

  bool captureHostFace()
  {
    if (dryRun() || guestIndex() > 1) {
      return true;
    }

    receptionist_interfaces::action::CaptureNamedProfile::Goal goal;
    goal.profile_id = "host";
    goal.role = "host";
    goal.name = "Host";
    auto result = callAction<receptionist_interfaces::action::CaptureNamedProfile>(
      "/appearance/capture_named_profile", goal, 3.0, 30.0);
    return result && result->success;
  }

  bool introduceGuest()
  {
    std::string guest_name;
    std::string favorite_drink;
    getStateMachine()->getGlobalSMData("guest_name", guest_name);
    getStateMachine()->getGlobalSMData("favorite_drink", favorite_drink);
    const auto fallback =
      "Please welcome " + (guest_name.empty() ? std::string("our guest") : guest_name) +
      ". Their favorite drink is " +
      (favorite_drink.empty() ? std::string("unknown") : favorite_drink) + ".";
    return speakWithAdvice("introduction", fallback);
  }

  bool findEmptyChair()
  {
    getStateMachine()->setGlobalSMData("selected_chair_id", std::string(""));
    if (dryRun()) {
      getStateMachine()->setGlobalSMData("selected_chair_id", std::string("dry_run_chair"));
      getStateMachine()->setGlobalSMData("selected_chair_waist_yaw_rad", 0.0);
      return true;
    }

    auto req = std::make_shared<receptionist_interfaces::srv::FindEmptyChair::Request>();
    auto resp = callService<receptionist_interfaces::srv::FindEmptyChair>(
      "/chair/find_empty_chair", req, 3.0, 5.0);
    if (!resp || !resp->success) {
      getStateMachine()->setGlobalSMData("last_error", std::string("no_empty_chair"));
      return true;
    }

    getStateMachine()->setGlobalSMData("selected_chair_id", resp->chair_id);
    getStateMachine()->setGlobalSMData(
      "selected_chair_waist_yaw_rad", static_cast<double>(resp->waist_yaw_rad));
    return true;
  }

  bool pointChair()
  {
    std::string chair_id;
    double waist_yaw = 0.0;
    getStateMachine()->getGlobalSMData("selected_chair_id", chair_id);
    getStateMachine()->getGlobalSMData("selected_chair_waist_yaw_rad", waist_yaw);
    if (chair_id.empty()) {
      return true;
    }
    if (dryRun()) {
      return true;
    }

    auto req = std::make_shared<receptionist_interfaces::srv::PointToSeat::Request>();
    req->chair_id = chair_id;
    req->waist_yaw_rad = static_cast<float>(waist_yaw);
    req->return_home = false;
    req->home_only = false;
    auto resp = callService<receptionist_interfaces::srv::PointToSeat>(
      "/chair/point_to_seat", req, 3.0, 10.0);
    return resp && resp->success;
  }

  bool askSit()
  {
    std::string chair_id;
    getStateMachine()->getGlobalSMData("selected_chair_id", chair_id);
    if (chair_id.empty()) {
      return speakText("Please remain near the host while I continue.");
    }
    return speakText("Please sit on the empty chair in front of me.");
  }

  bool markSeated()
  {
    std::string profile_id;
    getStateMachine()->getGlobalSMData("current_guest_committed_profile_id", profile_id);
    if (dryRun()) {
      return true;
    }

    auto req = std::make_shared<receptionist_interfaces::srv::MarkProfileSeated::Request>();
    req->profile_id = profile_id;
    auto resp = callService<receptionist_interfaces::srv::MarkProfileSeated>(
      "/guest_memory/mark_profile_seated", req, 2.0, 2.0);
    return resp && resp->success;
  }

  std::string step_name_;
};

class CbCheckGuestLoop : public smacc2::SmaccAsyncClientBehavior
{
public:
  void onEntry() override
  {
    int guest_index = 1;
    int target_guest_count = 2;
    getStateMachine()->getGlobalSMData("guest_index", guest_index);
    getStateMachine()->getGlobalSMData("target_guest_count", target_guest_count);

    if (guest_index < target_guest_count) {
      const int next_guest = guest_index + 1;
      getStateMachine()->setGlobalSMData("guest_index", next_guest);
      getStateMachine()->setGlobalSMData("current_guest_profile_id", std::string("current_guest"));
      getStateMachine()->setGlobalSMData("current_guest_committed_profile_id", std::string(""));
      getStateMachine()->setGlobalSMData("guest_name", std::string(""));
      getStateMachine()->setGlobalSMData("favorite_drink", std::string(""));
      RCLCPP_INFO(getLogger(), "Preparing receptionist guest %d.", next_guest);
      postEvent<events::EvNextGuest>();
      return;
    }

    RCLCPP_INFO(getLogger(), "All receptionist guests handled.");
    postEvent<events::EvAllGuestsDone>();
  }
};
}  // namespace client_behaviors

// Shutdown helper: delay SMACC2 and ROS shutdown until final-state onEntry has returned.
void requestSmaccStopAndShutdown(smacc2::ISmaccStateMachine * state_machine)
{
  // Detached worker: avoid terminating Boost statechart while it is still entering a state.
  std::thread(
    [state_machine]() {
      // Default guard: bypass fragile ROS/DDS destructor paths after a terminal SMACC2 state.
      bool force_process_exit = true;
      // Parameter read: let deployments request a conventional rclcpp shutdown if desired.
      if (state_machine != nullptr) {
        // Node access: state machine owns the SMACC2 ROS node.
        const auto node = state_machine->getNode();
        // Runtime override: preserve normal shutdown for debugging or custom supervisors.
        force_process_exit = node->get_parameter("force_process_exit_after_terminal_state").as_bool();
      }
      // Grace period: let SMACC2 finish the active transition before statechart teardown.
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      // SMACC2 stop: terminate active states before the scheduler is joined.
      if (state_machine != nullptr) {
        // Stop call: use SMACC2's own termination hook after the final state is fully entered.
        state_machine->stop();
      }
      // Grace period: let SMACC2 finish state exit callbacks before ROS shuts down.
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      // Hard terminal exit: avoids known Humble SMACC2/DDS teardown crashes after task completion.
      if (force_process_exit) {
        // Flush logs: preserve terminal-state messages before exiting the process.
        std::fflush(nullptr);
        // Exit code: report successful task completion to ros2 launch.
        std::_Exit(0);
      }
      // Guard: shutdown once so launch exits cleanly without double-shutdown noise.
      if (rclcpp::ok()) {
        // ROS shutdown: causes smacc2::run to return through the normal executor path.
        rclcpp::shutdown();
      }
    })
    .detach();
}

template<typename StateT, typename NextStateT>
struct LinearStepState : smacc2::SmaccState<StateT, SmReceptionist>
{
  using SmaccState = smacc2::SmaccState<StateT, SmReceptionist>;
  using StepSuccess = smacc2::EvCbSuccess<client_behaviors::CbReceptionistStep, orthogonals::OrTask>;
  using StepFailure = smacc2::EvCbFailure<client_behaviors::CbReceptionistStep, orthogonals::OrTask>;
  using reactions = boost::mpl::list<
    smacc2::Transition<StepSuccess, NextStateT, smacc2::SUCCESS>,
    smacc2::Transition<StepFailure, StAborted, smacc2::ABORT>>;
  using SmaccState::SmaccState;
};

struct StStartupGate : LinearStepState<StStartupGate, StInitHostMemory>
{
  using LinearStepState<StStartupGate, StInitHostMemory>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>(
      "startup_gate");
  }
};

struct StInitHostMemory : LinearStepState<StInitHostMemory, StWaitGuest>
{
  using LinearStepState<StInitHostMemory, StWaitGuest>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>(
      "init_host_memory");
  }
};

struct StWaitGuest : LinearStepState<StWaitGuest, StApproachGuest>
{
  using LinearStepState<StWaitGuest, StApproachGuest>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>("wait_guest");
  }
};

struct StApproachGuest : LinearStepState<StApproachGuest, StGreetGuest>
{
  using LinearStepState<StApproachGuest, StGreetGuest>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>(
      "approach_guest");
  }
};

struct StGreetGuest : LinearStepState<StGreetGuest, StCollectProfile>
{
  using LinearStepState<StGreetGuest, StCollectProfile>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>("greet_guest");
  }
};

struct StCollectProfile : LinearStepState<StCollectProfile, StTrackAppearance>
{
  using LinearStepState<StCollectProfile, StTrackAppearance>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>(
      "collect_profile");
  }
};

struct StTrackAppearance : LinearStepState<StTrackAppearance, StStoreGuest>
{
  using LinearStepState<StTrackAppearance, StStoreGuest>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>(
      "track_appearance");
  }
};

struct StStoreGuest : LinearStepState<StStoreGuest, StCommitGuest>
{
  using LinearStepState<StStoreGuest, StCommitGuest>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>("store_guest");
  }
};

struct StCommitGuest : LinearStepState<StCommitGuest, StGoLivingRoom>
{
  using LinearStepState<StCommitGuest, StGoLivingRoom>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>("commit_guest");
  }
};

struct StGoLivingRoom : LinearStepState<StGoLivingRoom, StResolveTargets>
{
  using LinearStepState<StGoLivingRoom, StResolveTargets>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>(
      "go_living_room");
  }
};

struct StResolveTargets : LinearStepState<StResolveTargets, StFaceTarget>
{
  using LinearStepState<StResolveTargets, StFaceTarget>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>(
      "resolve_targets");
  }
};

struct StFaceTarget : LinearStepState<StFaceTarget, StCaptureHostFace>
{
  using LinearStepState<StFaceTarget, StCaptureHostFace>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>("face_target");
  }
};

struct StCaptureHostFace : LinearStepState<StCaptureHostFace, StIntroduceGuest>
{
  using LinearStepState<StCaptureHostFace, StIntroduceGuest>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>(
      "capture_host_face");
  }
};

struct StIntroduceGuest : LinearStepState<StIntroduceGuest, StFindEmptyChair>
{
  using LinearStepState<StIntroduceGuest, StFindEmptyChair>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>(
      "introduce_guest");
  }
};

struct StFindEmptyChair : LinearStepState<StFindEmptyChair, StFaceChair>
{
  using LinearStepState<StFindEmptyChair, StFaceChair>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>(
      "find_empty_chair");
  }
};

struct StFaceChair : LinearStepState<StFaceChair, StPointChair>
{
  using LinearStepState<StFaceChair, StPointChair>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>("face_chair");
  }
};

struct StPointChair : LinearStepState<StPointChair, StAskSit>
{
  using LinearStepState<StPointChair, StAskSit>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>("point_chair");
  }
};

struct StAskSit : LinearStepState<StAskSit, StMarkSeated>
{
  using LinearStepState<StAskSit, StMarkSeated>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>("ask_sit");
  }
};

struct StMarkSeated : LinearStepState<StMarkSeated, StCheckLoop>
{
  using LinearStepState<StMarkSeated, StCheckLoop>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>("mark_seated");
  }
};

struct StCheckLoop : smacc2::SmaccState<StCheckLoop, SmReceptionist>
{
  using SmaccState::SmaccState;
  using reactions = boost::mpl::list<
    smacc2::Transition<events::EvNextGuest, StReturnReception, smacc2::SUCCESS>,
    smacc2::Transition<events::EvAllGuestsDone, StReturnStart, smacc2::SUCCESS>>;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbCheckGuestLoop>();
  }
};

struct StReturnReception : LinearStepState<StReturnReception, StWaitGuest>
{
  using LinearStepState<StReturnReception, StWaitGuest>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>(
      "return_reception");
  }
};

struct StReturnStart : LinearStepState<StReturnStart, StFinished>
{
  using LinearStepState<StReturnStart, StFinished>::LinearStepState;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbReceptionistStep>(
      "return_start");
  }
};

struct StFinished : smacc2::SmaccState<StFinished, SmReceptionist>
{
  using SmaccState::SmaccState;

  static void staticConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "SMACC2 receptionist task finished successfully.");
    requestSmaccStopAndShutdown(&getStateMachine());
  }
};

struct StAborted : smacc2::SmaccState<StAborted, SmReceptionist>
{
  using SmaccState::SmaccState;

  static void staticConfigure() {}

  void onEntry()
  {
    RCLCPP_ERROR(getLogger(), "SMACC2 receptionist task aborted safely.");
    requestSmaccStopAndShutdown(&getStateMachine());
  }
};
}  // namespace sm_receptionist

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  smacc2::run<sm_receptionist::SmReceptionist>();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
