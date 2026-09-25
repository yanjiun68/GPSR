#include <boost/mpl/list.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_ai_interfaces/action/speak_text.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <robot_ai_interfaces/action/collect_guest_profile.hpp>
#include <robot_ai_interfaces/action/navigate_named_pose.hpp>
#include <robot_ai_interfaces/msg/guest_appearance.hpp>
#include <robot_ai_interfaces/msg/person_profile.hpp>
#include <robot_ai_interfaces/srv/clear_current_guest.hpp>
#include <robot_ai_interfaces/srv/set_current_guest_profile.hpp>
#include <smacc2/smacc.hpp>
#include <std_msgs/msg/string.hpp>

#include "sm_receptionist/task_receptionist_process_utils.hpp"

namespace sm_receptionist::task_receptionist
{
namespace orthogonals
{
// Section: single sequential task orthogonal used by every client behavior.
struct OrTask : smacc2::Orthogonal<OrTask>
{
};
}  // namespace orthogonals

namespace events
{
// Section: loop-control events used after one complete guest cycle.
struct EvLoopAgain : boost::statechart::event<EvLoopAgain>
{
};

struct EvTaskComplete : boost::statechart::event<EvTaskComplete>
{
};
}  // namespace events

// Section: forward declarations for all exact-flow Task Receptionist states.
struct StStartupReadiness;
struct StWaitBeforeGuest;
struct StNavigateToWaypoint7;
struct StLaunchInsightFace;
struct StWaitForFace;
struct StCollectAppearance;
struct StWaitAfterFeatures;
struct StAskGuestInformation;
struct StSayFollowMe;
struct StNavigateToWaypoint6;
struct StSpeakFeatureSummary;
struct StSpeakGuestInformation;
struct StNavigateToWaypoint8;
struct StSaySeatMiddle;
struct StNavigateToWaypoint99;
struct StClearGuestData;
struct StCheckLoop;
struct StFinished;
struct StAborted;

// Section: state machine definition and shared global-data initialization.
struct SmTaskReceptionist : smacc2::SmaccStateMachineBase<SmTaskReceptionist, StStartupReadiness>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  // Special function: declare runtime parameters and seed the SMACC2 blackboard.
  void onInitialize() override
  {
    auto node = getNode();

    node->declare_parameter<bool>("dry_run", true);
    node->declare_parameter<int>("max_cycles", 2);
    node->declare_parameter<double>("wait_before_guest_sec", 3.0);
    node->declare_parameter<double>("post_feature_wait_sec", 2.0);
    node->declare_parameter<double>("startup_readiness_timeout_sec", 0.0);
    node->declare_parameter<std::string>("navigation_action_name", "/nav/navigate_named_pose");
    node->declare_parameter<std::string>("navigate_to_pose_action", "/navigate_to_pose");
    node->declare_parameter<std::string>("waypoint_7", "7");
    node->declare_parameter<std::string>("waypoint_6", "6");
    node->declare_parameter<std::string>("waypoint_8", "8");
    node->declare_parameter<std::string>("waypoint_99", "99");
    node->declare_parameter<std::string>("tts_action_name", "/tts/speak");
    node->declare_parameter<std::string>("dialogue_action_name", "/dialogue/collect_guest_profile");
    node->declare_parameter<std::string>("dialogue_mic_device", "auto");
    node->declare_parameter<double>("dialogue_record_sec", 4.0);
    node->declare_parameter<std::string>("dialogue_piper_model_path", "");
    node->declare_parameter<std::string>("dialogue_whisper_model", "base");
    node->declare_parameter<std::string>("dialogue_whisper_language", "en");
    node->declare_parameter<bool>("allow_dialogue_fallback", false);
    node->declare_parameter<bool>("store_profile_in_memory", true);
    node->declare_parameter<bool>("personal_test_mode", true);
    node->declare_parameter<std::string>("camera_index", "0");
    node->declare_parameter<std::string>("camera_backend", "realsense");
    node->declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
    node->declare_parameter<bool>("enable_depth", true);
    node->declare_parameter<std::string>("face_topic", "/perception/largest_face");
    node->declare_parameter<std::string>("appearance_topic", "/appearance/current_estimate");
    node->declare_parameter<double>("face_wait_timeout_sec", 10.0);
    node->declare_parameter<double>("appearance_wait_timeout_sec", 10.0);
    node->declare_parameter<double>("collection_duration_sec", 2.0);
    node->declare_parameter<int>("min_stable_samples", 3);
    node->declare_parameter<bool>("show_appearance_gui", true);
    node->declare_parameter<double>("stream_info_period_sec", 1.0);
    node->declare_parameter<bool>("force_process_exit_after_terminal_state", true);

    setGlobalSMData("dry_run", node->get_parameter("dry_run").as_bool());
    setGlobalSMData("max_cycles", static_cast<int>(node->get_parameter("max_cycles").as_int()));
    setGlobalSMData("cycle_index", 1);
    setGlobalSMData("guest_profile_id", std::string("current_guest"));
    setGlobalSMData("guest_name", std::string{});
    setGlobalSMData("favorite_drink", std::string{});
    setGlobalSMData("guest_gender", std::string{});
    setGlobalSMData("guest_hometown", std::string{});
    setGlobalSMData("guest_info_text", std::string{});
    setGlobalSMData("summary_sample_count", 0);
    setGlobalSMData("shirt_color", std::string{});
    setGlobalSMData("shirt_color_confidence", 0.0);
    setGlobalSMData(
      "glasses_state",
      static_cast<int>(robot_ai_interfaces::msg::GuestAppearance::UNKNOWN));
    setGlobalSMData("glasses_confidence", 0.0);
    setGlobalSMData(
      "hat_state",
      static_cast<int>(robot_ai_interfaces::msg::GuestAppearance::UNKNOWN));
    setGlobalSMData("hat_confidence", 0.0);
    setGlobalSMData(
      "face_mask_state",
      static_cast<int>(robot_ai_interfaces::msg::GuestAppearance::UNKNOWN));
    setGlobalSMData("face_mask_confidence", 0.0);
    setGlobalSMData("feature_summary_text", std::string{});
    setGlobalSMData("active_insightface_pgid", static_cast<std::int64_t>(-1));
    setGlobalSMData("last_error", std::string{});

    process_utils::clearInsightFacePgid();
    createOrthogonal<orthogonals::OrTask>();

    RCLCPP_INFO(
      getLogger(),
      "Task Receptionist initialized: dry_run=%s max_cycles=%ld waypoints=[%s,%s,%s,%s]",
      node->get_parameter("dry_run").as_bool() ? "true" : "false",
      node->get_parameter("max_cycles").as_int(),
      node->get_parameter("waypoint_7").as_string().c_str(),
      node->get_parameter("waypoint_6").as_string().c_str(),
      node->get_parameter("waypoint_8").as_string().c_str(),
      node->get_parameter("waypoint_99").as_string().c_str());
  }
};

namespace client_behaviors
{
using CollectGuestProfile = robot_ai_interfaces::action::CollectGuestProfile;
using GuestAppearance = robot_ai_interfaces::msg::GuestAppearance;
using NavigateNamedPose = robot_ai_interfaces::action::NavigateNamedPose;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using PersonProfile = robot_ai_interfaces::msg::PersonProfile;
using TtsSpeakText = robot_ai_interfaces::action::SpeakText;

// Section: small formatting helpers shared by launch, speech, and appearance summary logic.
std::chrono::milliseconds secondsToMilliseconds(double seconds)
{
  const auto bounded_seconds = std::max(0.0, seconds);
  return std::chrono::milliseconds(static_cast<int>(bounded_seconds * 1000.0));
}

// Special helper: render a bool as a ROS launch string.
std::string boolString(bool value)
{
  return value ? "true" : "false";
}

// Special helper: render a double with stable precision for launch arguments.
std::string formatDouble(double value)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << value;
  return out.str();
}

// Special helper: build one safely quoted ROS launch argument.
std::string launchArg(const std::string & name, const std::string & value)
{
  return process_utils::shellQuote(name + ":=" + value);
}

// Special helper: normalize appearance color strings before voting and speech.
std::string normalizeColor(std::string color)
{
  color.erase(
    color.begin(),
    std::find_if(
      color.begin(),
      color.end(),
      [](unsigned char ch) {
        return !std::isspace(ch);
      }));
  color.erase(
    std::find_if(
      color.rbegin(),
      color.rend(),
      [](unsigned char ch) {
        return !std::isspace(ch);
      }).base(),
    color.end());
  std::transform(
    color.begin(),
    color.end(),
    color.begin(),
    [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });

  if (color == "unknown" || color == "none" || color == "n/a") {
    return {};
  }
  return color;
}

// Special helper: construct the InsightFace appearance pipeline command.
std::string buildInsightFaceLaunchCommand(
  bool personal_test_mode,
  const std::string & camera_index,
  const std::string & camera_backend,
  const std::string & image_topic,
  bool enable_depth,
  const std::string & face_topic,
  const std::string & appearance_topic,
  bool show_appearance_gui,
  double stream_info_period_sec)
{
  return std::string("ros2 launch face_detection appearance_pipeline.launch.py ") +
         launchArg("personal_test_mode", boolString(personal_test_mode)) + " " +
         launchArg("camera_index", camera_index) + " " +
         launchArg("camera_backend", camera_backend) + " " +
         launchArg("image_topic", image_topic) + " " +
         launchArg("enable_depth", boolString(enable_depth)) + " " +
         launchArg("face_topic", face_topic) + " " +
         launchArg("appearance_topic", appearance_topic) + " " +
         launchArg("show_appearance_gui", boolString(show_appearance_gui)) + " " +
         launchArg("show_stream_info_overlay", "true") + " " +
         launchArg("log_video_stats", "true") + " " +
         launchArg("stream_info_period_sec", formatDouble(stream_info_period_sec)) + " " +
         launchArg("enable_profile_capture", "false") + " " +
         launchArg("enable_identification", "false");
}

// Section: vote structures used to summarize several appearance samples.
struct AttributeVote
{
  int yes_count{0};
  int no_count{0};
  int unknown_count{0};
  double yes_confidence{0.0};
  double no_confidence{0.0};
};

// Special helper: accumulate one binary/unknown attribute sample.
void addAttributeSample(AttributeVote & vote, std::uint8_t state, double confidence)
{
  if (state == GuestAppearance::YES) {
    vote.yes_count += 1;
    vote.yes_confidence = std::max(vote.yes_confidence, confidence);
    return;
  }

  if (state == GuestAppearance::NO) {
    vote.no_count += 1;
    vote.no_confidence = std::max(vote.no_confidence, confidence);
    return;
  }

  vote.unknown_count += 1;
}

// Special helper: choose the final state for a voted attribute.
std::pair<int, double> chooseAttributeState(const AttributeVote & vote)
{
  if (vote.yes_count > 0 && vote.yes_count >= vote.no_count) {
    return {static_cast<int>(GuestAppearance::YES), vote.yes_confidence};
  }

  if (vote.no_count > 0 && vote.no_count > vote.yes_count) {
    return {static_cast<int>(GuestAppearance::NO), vote.no_confidence};
  }

  return {static_cast<int>(GuestAppearance::UNKNOWN), 0.0};
}

// Section: aggregated feature output stored in SMACC2 global data.
struct AggregatedAppearance
{
  int sample_count{0};
  std::string shirt_color;
  double shirt_color_confidence{0.0};
  int glasses_state{static_cast<int>(GuestAppearance::UNKNOWN)};
  double glasses_confidence{0.0};
  int hat_state{static_cast<int>(GuestAppearance::UNKNOWN)};
  double hat_confidence{0.0};
  int face_mask_state{static_cast<int>(GuestAppearance::UNKNOWN)};
  double face_mask_confidence{0.0};
};

// Special helper: vote across all collected appearance messages.
AggregatedAppearance aggregateAppearance(const std::vector<GuestAppearance> & samples)
{
  AggregatedAppearance aggregate;
  aggregate.sample_count = static_cast<int>(samples.size());

  std::map<std::string, std::pair<double, int>> color_scores;
  AttributeVote glasses_vote;
  AttributeVote hat_vote;
  AttributeVote face_mask_vote;

  for (const auto & sample : samples) {
    const auto color = normalizeColor(sample.shirt_color);
    if (!color.empty()) {
      auto & score = color_scores[color];
      score.first += static_cast<double>(sample.shirt_color_confidence);
      score.second += 1;
    }

    addAttributeSample(glasses_vote, sample.glasses, sample.glasses_confidence);
    addAttributeSample(hat_vote, sample.hat, sample.hat_confidence);
    addAttributeSample(face_mask_vote, sample.face_mask, sample.face_mask_confidence);
  }

  double best_color_average = -1.0;
  int best_color_count = -1;
  for (const auto & [color, score] : color_scores) {
    const double average = score.second > 0 ? score.first / static_cast<double>(score.second) : 0.0;
    if (score.second > best_color_count ||
      (score.second == best_color_count && average > best_color_average))
    {
      aggregate.shirt_color = color;
      aggregate.shirt_color_confidence = average;
      best_color_average = average;
      best_color_count = score.second;
    }
  }

  const auto [glasses_state, glasses_confidence] = chooseAttributeState(glasses_vote);
  const auto [hat_state, hat_confidence] = chooseAttributeState(hat_vote);
  const auto [face_mask_state, face_mask_confidence] = chooseAttributeState(face_mask_vote);
  aggregate.glasses_state = glasses_state;
  aggregate.glasses_confidence = glasses_confidence;
  aggregate.hat_state = hat_state;
  aggregate.hat_confidence = hat_confidence;
  aggregate.face_mask_state = face_mask_state;
  aggregate.face_mask_confidence = face_mask_confidence;

  return aggregate;
}

// Special helper: turn a YES/NO/UNKNOWN state into one sentence.
std::string describeAttribute(
  int state,
  const std::string & yes_sentence,
  const std::string & no_sentence,
  const std::string & unknown_sentence)
{
  if (state == GuestAppearance::YES) {
    return yes_sentence;
  }
  if (state == GuestAppearance::NO) {
    return no_sentence;
  }
  return unknown_sentence;
}

// Special helper: build the speech text used to read InsightFace features aloud.
std::string buildFeatureSummaryText(
  const std::string & shirt_color,
  int glasses_state,
  int hat_state,
  int face_mask_state)
{
  std::ostringstream text;

  if (!shirt_color.empty()) {
    text << "I detected a guest wearing a " << shirt_color << " shirt. ";
  } else {
    text << "I detected the guest, but I could not confidently determine the shirt color. ";
  }

  text << describeAttribute(
    glasses_state,
    "The guest is wearing glasses. ",
    "The guest is not wearing glasses. ",
    "I cannot confidently determine whether the guest is wearing glasses. ");
  text << describeAttribute(
    hat_state,
    "The guest is wearing a hat. ",
    "The guest is not wearing a hat. ",
    "I cannot confidently determine whether the guest is wearing a hat. ");
  text << describeAttribute(
    face_mask_state,
    "The guest is wearing a face mask.",
    "The guest is not wearing a face mask.",
    "I cannot confidently determine whether the guest is wearing a face mask.");

  return text.str();
}

// Special helper: provide a deterministic dry-run profile for integration checks.
PersonProfile fallbackProfile(int cycle_index)
{
  PersonProfile profile;
  profile.profile_id = "current_guest";
  profile.role = "guest";
  profile.name = "Guest " + std::to_string(cycle_index);
  profile.age = 0;
  profile.gender = "unknown";
  profile.favorite_drink = "water";
  profile.hometown = "";
  profile.is_confirmed = true;
  profile.confidence_score = 0.5;
  return profile;
}

// Special helper: build the speech text used to read collected guest information aloud.
std::string buildGuestInformationText(const PersonProfile & profile)
{
  std::ostringstream text;

  text << "The guest information I recorded is: ";
  if (!profile.name.empty()) {
    text << "name, " << profile.name << ". ";
  } else {
    text << "name, unknown. ";
  }

  if (!profile.favorite_drink.empty()) {
    text << "Favorite drink, " << profile.favorite_drink << ". ";
  } else {
    text << "Favorite drink, unknown. ";
  }

  if (!profile.gender.empty() && profile.gender != "unknown") {
    text << "Gender, " << profile.gender << ". ";
  }

  if (!profile.hometown.empty()) {
    text << "Hometown, " << profile.hometown << ".";
  }

  return text.str();
}

// Section: one asynchronous behavior dispatches the named flow step for each state.
class CbTaskReceptionistStep : public smacc2::SmaccAsyncClientBehavior
{
public:
  // Special function: store the exact step name configured by the state.
  explicit CbTaskReceptionistStep(std::string step_name)
  : step_name_(std::move(step_name))
  {
  }

  // Special function: run the configured step and post the corresponding SMACC2 event.
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
  // Special helper: dispatch one configured state-machine step.
  bool runStep()
  {
    if (step_name_ == "startup_readiness") {
      return waitForStartupReadiness();
    }
    if (step_name_ == "wait_before_guest") {
      return waitSeconds(doubleParam("wait_before_guest_sec", 3.0), "initial receptionist wait");
    }
    if (step_name_ == "navigate_waypoint_7") {
      return navigateToParam("waypoint_7", "7");
    }
    if (step_name_ == "launch_insightface") {
      return launchInsightFace();
    }
    if (step_name_ == "wait_for_face") {
      return waitForFace();
    }
    if (step_name_ == "collect_appearance") {
      return collectAppearance();
    }
    if (step_name_ == "wait_after_features") {
      return waitSeconds(doubleParam("post_feature_wait_sec", 2.0), "post-feature wait");
    }
    if (step_name_ == "ask_guest_information") {
      return askGuestInformation();
    }
    if (step_name_ == "say_follow_me") {
      return speakText("Follow me on my right");
    }
    if (step_name_ == "navigate_waypoint_6") {
      return navigateToParam("waypoint_6", "6");
    }
    if (step_name_ == "speak_feature_summary") {
      return speakFeatureSummary();
    }
    if (step_name_ == "speak_guest_information") {
      return speakGuestInformation();
    }
    if (step_name_ == "navigate_waypoint_8") {
      return navigateToParam("waypoint_8", "8");
    }
    if (step_name_ == "say_seat_middle") {
      return speakText("please seat on the seat in the middle");
    }
    if (step_name_ == "navigate_waypoint_99") {
      return navigateToParam("waypoint_99", "99");
    }
    if (step_name_ == "clear_guest_data") {
      return clearGuestData();
    }

    RCLCPP_ERROR(getLogger(), "Unknown Task Receptionist step: %s", step_name_.c_str());
    return false;
  }

  // Special helper: return whether the state machine should avoid hardware side effects.
  bool dryRun()
  {
    bool dry_run = true;
    getStateMachine()->getGlobalSMData("dry_run", dry_run);
    return dry_run;
  }

  // Special helper: read the current loop cycle from the SMACC2 blackboard.
  int cycleIndex()
  {
    int cycle_index = 1;
    getStateMachine()->getGlobalSMData("cycle_index", cycle_index);
    return cycle_index;
  }

  // Special helper: declare a string parameter if needed and return its value.
  std::string stringParam(const std::string & name, const std::string & fallback)
  {
    auto node = getNode();
    if (!node->has_parameter(name)) {
      node->declare_parameter<std::string>(name, fallback);
    }
    return node->get_parameter(name).as_string();
  }

  // Special helper: declare a floating-point parameter if needed and return its value.
  double doubleParam(const std::string & name, double fallback)
  {
    auto node = getNode();
    if (!node->has_parameter(name)) {
      node->declare_parameter<double>(name, fallback);
    }
    return node->get_parameter(name).as_double();
  }

  // Special helper: declare an integer parameter if needed and return its value.
  int intParam(const std::string & name, int fallback)
  {
    auto node = getNode();
    if (!node->has_parameter(name)) {
      node->declare_parameter<int>(name, fallback);
    }
    return static_cast<int>(node->get_parameter(name).as_int());
  }

  // Special helper: declare a boolean parameter if needed and return its value.
  bool boolParam(const std::string & name, bool fallback)
  {
    auto node = getNode();
    if (!node->has_parameter(name)) {
      node->declare_parameter<bool>(name, fallback);
    }
    return node->get_parameter(name).as_bool();
  }

  // Special helper: join missing readiness dependency names for readable diagnostics.
  std::string joinNames(const std::vector<std::string> & names)
  {
    std::ostringstream out;
    for (std::size_t i = 0; i < names.size(); ++i) {
      if (i > 0) {
        out << ", ";
      }
      out << names[i];
    }
    return out.str();
  }

  // Special helper: check whether a ROS action server is currently discoverable.
  template<typename ActionT>
  bool actionReady(const std::string & action_name, double wait_sec)
  {
    auto client = rclcpp_action::create_client<ActionT>(getNode(), action_name);
    return client->wait_for_action_server(secondsToMilliseconds(wait_sec));
  }

  // Special helper: check whether a ROS service server is currently discoverable.
  template<typename ServiceT>
  bool serviceReady(const std::string & service_name, double wait_sec)
  {
    auto client = getNode()->create_client<ServiceT>(service_name);
    return client->wait_for_service(secondsToMilliseconds(wait_sec));
  }

  // Special helper: call a ROS service with bounded server and response waits.
  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr callService(
    const std::string & service_name,
    const typename ServiceT::Request::SharedPtr & request,
    double wait_sec,
    double response_sec)
  {
    auto node = getNode();
    auto client = node->create_client<ServiceT>(service_name);
    if (!client->wait_for_service(secondsToMilliseconds(wait_sec))) {
      RCLCPP_WARN(getLogger(), "Service unavailable: %s", service_name.c_str());
      return nullptr;
    }

    auto future = client->async_send_request(request);
    if (future.wait_for(secondsToMilliseconds(response_sec)) != std::future_status::ready) {
      RCLCPP_WARN(getLogger(), "Service timed out: %s", service_name.c_str());
      return nullptr;
    }

    return future.get();
  }

  // Special helper: call a ROS action with bounded server, accept, and result waits.
  template<typename ActionT>
  typename ActionT::Result::SharedPtr callAction(
    const std::string & action_name,
    const typename ActionT::Goal & goal,
    double wait_sec,
    double result_sec)
  {
    auto node = getNode();
    auto client = rclcpp_action::create_client<ActionT>(node, action_name);
    if (!client->wait_for_action_server(secondsToMilliseconds(wait_sec))) {
      RCLCPP_WARN(getLogger(), "Action unavailable: %s", action_name.c_str());
      return nullptr;
    }

    auto goal_future = client->async_send_goal(goal);
    if (goal_future.wait_for(secondsToMilliseconds(wait_sec)) != std::future_status::ready) {
      RCLCPP_WARN(getLogger(), "Action goal timed out: %s", action_name.c_str());
      return nullptr;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_WARN(getLogger(), "Action goal rejected: %s", action_name.c_str());
      return nullptr;
    }

    auto result_future = client->async_get_result(goal_handle);
    if (result_future.wait_for(secondsToMilliseconds(result_sec)) != std::future_status::ready) {
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

  // Special helper: wait for an exact duration while still allowing shutdown.
  bool waitSeconds(double seconds, const std::string & label)
  {
    RCLCPP_INFO(getLogger(), "Waiting %.2f seconds for %s.", seconds, label.c_str());
    const auto deadline = std::chrono::steady_clock::now() + secondsToMilliseconds(seconds);
    while (rclcpp::ok() && !isShutdownRequested() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    return rclcpp::ok() && !isShutdownRequested();
  }

  // Special helper: wait until all startup-critical ROS interfaces are usable.
  bool waitForStartupReadiness()
  {
    const auto timeout_sec = doubleParam("startup_readiness_timeout_sec", 0.0);
    const auto navigation_action = stringParam("navigation_action_name", "/nav/navigate_named_pose");
    const auto tts_action = stringParam("tts_action_name", "/tts/speak");
    const auto dialogue_action = stringParam(
      "dialogue_action_name",
      "/dialogue/collect_guest_profile");
    const auto start = std::chrono::steady_clock::now();
    auto node = getNode();

    RCLCPP_INFO(
      getLogger(),
      "Waiting for startup readiness. timeout=%.2f sec",
      timeout_sec);

    while (rclcpp::ok() && !isShutdownRequested()) {
      std::vector<std::string> missing;

      if (!actionReady<NavigateNamedPose>(navigation_action, 0.2)) {
        missing.push_back(navigation_action);
      }
      if (!actionReady<TtsSpeakText>(tts_action, 0.2)) {
        missing.push_back(tts_action);
      }
      if (!actionReady<CollectGuestProfile>(dialogue_action, 0.2)) {
        missing.push_back(dialogue_action);
      }
      if (boolParam("store_profile_in_memory", true)) {
        if (!serviceReady<robot_ai_interfaces::srv::SetCurrentGuestProfile>(
            "/guest_memory/set_current_guest_profile",
            0.2))
        {
          missing.push_back("/guest_memory/set_current_guest_profile");
        }
        if (!serviceReady<robot_ai_interfaces::srv::ClearCurrentGuest>(
            "/guest_memory/clear_current_guest",
            0.2))
        {
          missing.push_back("/guest_memory/clear_current_guest");
        }
      }
      if (missing.empty()) {
        RCLCPP_INFO(getLogger(), "Startup readiness passed.");
        return true;
      }

      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      if (timeout_sec > 0.0 && elapsed >= timeout_sec) {
        const auto missing_text = joinNames(missing);
        getStateMachine()->setGlobalSMData(
          "last_error",
          std::string("startup_readiness_failed: missing ") + missing_text);
        RCLCPP_ERROR(getLogger(), "Startup readiness failed. Missing: %s", missing_text.c_str());
        return false;
      }

      RCLCPP_WARN_THROTTLE(
        getLogger(),
        *node->get_clock(),
        5000,
        "Waiting for startup dependencies: %s",
        joinNames(missing).c_str());
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    getStateMachine()->setGlobalSMData("last_error", std::string("startup_readiness_interrupted"));
    return false;
  }

  // Special helper: navigate to a named waypoint through the existing action goal contract.
  bool navigateToParam(const std::string & param_name, const std::string & fallback)
  {
    const auto target = stringParam(param_name, fallback);
    if (dryRun()) {
      RCLCPP_INFO(getLogger(), "Dry run: would navigate to waypoint target '%s'.", target.c_str());
      return true;
    }

    NavigateNamedPose::Goal goal;
    goal.target_name = target;
    const auto action_name = stringParam("navigation_action_name", "/nav/navigate_named_pose");
    auto result = callAction<NavigateNamedPose>(action_name, goal, 5.0, 180.0);
    return result && result->success;
  }

  // Special helper: speak one phrase through the configured my_tts_system TTS action.
  bool speakText(const std::string & text)
  {
    if (dryRun()) {
      RCLCPP_INFO(getLogger(), "Dry run speech: %s", text.c_str());
      return true;
    }

    TtsSpeakText::Goal goal;
    goal.text = text;
    const auto action_name = stringParam("tts_action_name", "/tts/speak");
    auto result = callAction<TtsSpeakText>(action_name, goal, 5.0, 60.0);
    if (!result || !result->success) {
      RCLCPP_ERROR(
        getLogger(),
        "Speech action failed through %s: %s",
        action_name.c_str(),
        result ? result->message.c_str() : "no result");
      return false;
    }
    return true;
  }

  // Special helper: start the InsightFace appearance pipeline only for feature capture.
  bool launchInsightFace()
  {
    if (dryRun()) {
      RCLCPP_INFO(getLogger(), "Dry run: skipping InsightFace launch.");
      return true;
    }

    process_utils::stopRegisteredInsightFace(getLogger());

    const auto command = buildInsightFaceLaunchCommand(
      boolParam("personal_test_mode", true),
      stringParam("camera_index", "0"),
      stringParam("camera_backend", "realsense"),
      stringParam("image_topic", "/camera/camera/color/image_raw"),
      boolParam("enable_depth", true),
      stringParam("face_topic", "/perception/largest_face"),
      stringParam("appearance_topic", "/appearance/current_estimate"),
      boolParam("show_appearance_gui", true),
      doubleParam("stream_info_period_sec", 1.0));

    const auto pid = process_utils::launchDetachedProcess(command);
    if (pid <= 0) {
      RCLCPP_ERROR(getLogger(), "Failed to launch InsightFace appearance pipeline.");
      return false;
    }

    process_utils::writeInsightFacePgid(pid);
    getStateMachine()->setGlobalSMData("active_insightface_pgid", static_cast<std::int64_t>(pid));

    RCLCPP_INFO(
      getLogger(),
      "Started InsightFace process group %d. Command: %s",
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
      return false;
    }

    return true;
  }

  // Special helper: wait until the face JSON stream reports a detected face.
  bool waitForFace()
  {
    if (dryRun()) {
      RCLCPP_INFO(getLogger(), "Dry run: treating face detection as successful.");
      return true;
    }

    const auto face_topic = stringParam("face_topic", "/perception/largest_face");
    const auto timeout_sec = doubleParam("face_wait_timeout_sec", 10.0);
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
      "Waiting for a detected face on %s for up to %.2f seconds.",
      face_topic.c_str(),
      timeout_sec);

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !isShutdownRequested() &&
      !face_detected.load(std::memory_order_relaxed))
    {
      std::int64_t pgid = -1;
      getStateMachine()->getGlobalSMData("active_insightface_pgid", pgid);
      if (pgid > 0 && !process_utils::processGroupRunning(static_cast<pid_t>(pgid))) {
        RCLCPP_ERROR(getLogger(), "InsightFace exited before face detection.");
        return false;
      }

      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      if (timeout_sec > 0.0 && elapsed >= timeout_sec) {
        RCLCPP_ERROR(getLogger(), "Timed out waiting for a detected face.");
        return false;
      }

      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    return face_detected.load(std::memory_order_relaxed);
  }

  // Special helper: collect and aggregate appearance features from the InsightFace stream.
  bool collectAppearance()
  {
    if (dryRun()) {
      getStateMachine()->setGlobalSMData("summary_sample_count", 1);
      getStateMachine()->setGlobalSMData("shirt_color", std::string{});
      getStateMachine()->setGlobalSMData(
        "glasses_state",
        static_cast<int>(GuestAppearance::UNKNOWN));
      getStateMachine()->setGlobalSMData("hat_state", static_cast<int>(GuestAppearance::UNKNOWN));
      getStateMachine()->setGlobalSMData(
        "face_mask_state",
        static_cast<int>(GuestAppearance::UNKNOWN));
      getStateMachine()->setGlobalSMData(
        "feature_summary_text",
        buildFeatureSummaryText(
          "",
          static_cast<int>(GuestAppearance::UNKNOWN),
          static_cast<int>(GuestAppearance::UNKNOWN),
          static_cast<int>(GuestAppearance::UNKNOWN)));
      RCLCPP_INFO(getLogger(), "Dry run: stored unknown InsightFace feature summary.");
      return true;
    }

    const auto appearance_topic = stringParam("appearance_topic", "/appearance/current_estimate");
    const auto timeout_sec = doubleParam("appearance_wait_timeout_sec", 10.0);
    const auto collection_duration_sec = doubleParam("collection_duration_sec", 2.0);
    const auto min_stable_samples = intParam("min_stable_samples", 3);

    auto node = getNode();
    std::mutex mutex;
    std::vector<GuestAppearance> samples;
    auto first_sample_time = std::chrono::steady_clock::time_point{};
    auto subscription = node->create_subscription<GuestAppearance>(
      appearance_topic,
      10,
      [&mutex, &samples, &first_sample_time](const GuestAppearance::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        if (samples.empty()) {
          first_sample_time = std::chrono::steady_clock::now();
        }
        samples.push_back(*msg);
      });

    RCLCPP_INFO(
      getLogger(),
      "Collecting appearance samples on %s for %.2f seconds after first sample.",
      appearance_topic.c_str(),
      collection_duration_sec);

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !isShutdownRequested()) {
      std::int64_t pgid = -1;
      getStateMachine()->getGlobalSMData("active_insightface_pgid", pgid);
      if (pgid > 0 && !process_utils::processGroupRunning(static_cast<pid_t>(pgid))) {
        RCLCPP_ERROR(getLogger(), "InsightFace exited during appearance collection.");
        return false;
      }

      bool enough_samples = false;
      bool collection_window_elapsed = false;
      int sample_count = 0;
      {
        std::lock_guard<std::mutex> lock(mutex);
        sample_count = static_cast<int>(samples.size());
        enough_samples = sample_count >= std::max(1, min_stable_samples);
        if (!samples.empty()) {
          const auto elapsed_collection = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - first_sample_time).count();
          collection_window_elapsed = elapsed_collection >=
                                      std::max(0.0, collection_duration_sec);
        }
      }

      if (enough_samples && collection_window_elapsed) {
        break;
      }

      const auto elapsed_total = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      if (timeout_sec > 0.0 && elapsed_total >= timeout_sec) {
        if (sample_count <= 0) {
          RCLCPP_ERROR(getLogger(), "Timed out waiting for appearance samples.");
          return false;
        }
        RCLCPP_WARN(
          getLogger(),
          "Using %d appearance sample(s) after timeout.",
          sample_count);
        break;
      }

      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    std::vector<GuestAppearance> snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex);
      snapshot = samples;
    }

    if (snapshot.empty()) {
      RCLCPP_ERROR(getLogger(), "No appearance samples were collected.");
      return false;
    }

    const auto aggregate = aggregateAppearance(snapshot);
    const auto summary_text = buildFeatureSummaryText(
      aggregate.shirt_color,
      aggregate.glasses_state,
      aggregate.hat_state,
      aggregate.face_mask_state);

    getStateMachine()->setGlobalSMData("summary_sample_count", aggregate.sample_count);
    getStateMachine()->setGlobalSMData("shirt_color", aggregate.shirt_color);
    getStateMachine()->setGlobalSMData("shirt_color_confidence", aggregate.shirt_color_confidence);
    getStateMachine()->setGlobalSMData("glasses_state", aggregate.glasses_state);
    getStateMachine()->setGlobalSMData("glasses_confidence", aggregate.glasses_confidence);
    getStateMachine()->setGlobalSMData("hat_state", aggregate.hat_state);
    getStateMachine()->setGlobalSMData("hat_confidence", aggregate.hat_confidence);
    getStateMachine()->setGlobalSMData("face_mask_state", aggregate.face_mask_state);
    getStateMachine()->setGlobalSMData("face_mask_confidence", aggregate.face_mask_confidence);
    getStateMachine()->setGlobalSMData("feature_summary_text", summary_text);

    process_utils::stopRegisteredInsightFace(getLogger());
    getStateMachine()->setGlobalSMData("active_insightface_pgid", static_cast<std::int64_t>(-1));

    RCLCPP_INFO(
      getLogger(),
      "Stored feature summary from %d sample(s): %s",
      aggregate.sample_count,
      summary_text.c_str());
    return true;
  }

  // Special helper: send the guest-dialogue action goal and save the returned profile.
  bool askGuestInformation()
  {
    PersonProfile profile = fallbackProfile(cycleIndex());

    if (!dryRun()) {
      CollectGuestProfile::Goal goal;
      goal.mic_device = stringParam("dialogue_mic_device", "auto");
      goal.record_sec = static_cast<float>(doubleParam("dialogue_record_sec", 4.0));
      goal.piper_model_path = stringParam("dialogue_piper_model_path", "");
      goal.whisper_model = stringParam("dialogue_whisper_model", "base");
      goal.whisper_language = stringParam("dialogue_whisper_language", "en");

      const auto action_name = stringParam(
        "dialogue_action_name",
        "/dialogue/collect_guest_profile");
      auto result = callAction<CollectGuestProfile>(action_name, goal, 5.0, 90.0);
      if (result && result->success) {
        profile = result->profile;
      } else if (!boolParam("allow_dialogue_fallback", false)) {
        getStateMachine()->setGlobalSMData(
          "last_error",
          std::string("dialogue_profile_collection_failed"));
        return false;
      } else {
        RCLCPP_WARN(getLogger(), "Dialogue failed; using fallback guest profile.");
      }
    }

    profile.profile_id = "current_guest";
    profile.role = "guest";
    profile.stamp = getNode()->now();

    const auto guest_info_text = buildGuestInformationText(profile);
    getStateMachine()->setGlobalSMData("guest_profile_id", profile.profile_id);
    getStateMachine()->setGlobalSMData("guest_name", profile.name);
    getStateMachine()->setGlobalSMData("favorite_drink", profile.favorite_drink);
    getStateMachine()->setGlobalSMData("guest_gender", profile.gender);
    getStateMachine()->setGlobalSMData("guest_hometown", profile.hometown);
    getStateMachine()->setGlobalSMData("guest_info_text", guest_info_text);

    if (dryRun() || !boolParam("store_profile_in_memory", true)) {
      RCLCPP_INFO(getLogger(), "Recorded guest info: %s", guest_info_text.c_str());
      return true;
    }

    auto req =
      std::make_shared<robot_ai_interfaces::srv::SetCurrentGuestProfile::Request>();
    req->profile = profile;
    auto resp = callService<robot_ai_interfaces::srv::SetCurrentGuestProfile>(
      "/guest_memory/set_current_guest_profile", req, 3.0, 3.0);
    if (!resp || !resp->success) {
      RCLCPP_ERROR(
        getLogger(),
        "Failed to save current guest profile: %s",
        resp ? resp->message.c_str() : "no response");
      return false;
    }

    return true;
  }

  // Special helper: read the stored feature summary aloud.
  bool speakFeatureSummary()
  {
    std::string text;
    getStateMachine()->getGlobalSMData("feature_summary_text", text);
    if (text.empty()) {
      RCLCPP_ERROR(getLogger(), "Feature summary text is empty.");
      return false;
    }
    return speakText(text);
  }

  // Special helper: read the stored guest profile summary aloud.
  bool speakGuestInformation()
  {
    std::string text;
    getStateMachine()->getGlobalSMData("guest_info_text", text);
    if (text.empty()) {
      RCLCPP_ERROR(getLogger(), "Guest information text is empty.");
      return false;
    }
    return speakText(text);
  }

  // Special helper: clear temporary guest profile and appearance data before the next cycle.
  bool clearGuestData()
  {
    process_utils::stopRegisteredInsightFace(getLogger());
    getStateMachine()->setGlobalSMData("active_insightface_pgid", static_cast<std::int64_t>(-1));

    bool memory_cleared = true;
    if (!dryRun() && boolParam("store_profile_in_memory", true)) {
      auto req = std::make_shared<robot_ai_interfaces::srv::ClearCurrentGuest::Request>();
      auto resp = callService<robot_ai_interfaces::srv::ClearCurrentGuest>(
        "/guest_memory/clear_current_guest", req, 3.0, 3.0);
      memory_cleared = resp && resp->success;
      if (!memory_cleared) {
        RCLCPP_ERROR(
          getLogger(),
          "Failed to clear current guest memory: %s",
          resp ? resp->message.c_str() : "no response");
      }
    }

    getStateMachine()->setGlobalSMData("guest_profile_id", std::string("current_guest"));
    getStateMachine()->setGlobalSMData("guest_name", std::string{});
    getStateMachine()->setGlobalSMData("favorite_drink", std::string{});
    getStateMachine()->setGlobalSMData("guest_gender", std::string{});
    getStateMachine()->setGlobalSMData("guest_hometown", std::string{});
    getStateMachine()->setGlobalSMData("guest_info_text", std::string{});
    getStateMachine()->setGlobalSMData("summary_sample_count", 0);
    getStateMachine()->setGlobalSMData("shirt_color", std::string{});
    getStateMachine()->setGlobalSMData("shirt_color_confidence", 0.0);
    getStateMachine()->setGlobalSMData(
      "glasses_state",
      static_cast<int>(GuestAppearance::UNKNOWN));
    getStateMachine()->setGlobalSMData("glasses_confidence", 0.0);
    getStateMachine()->setGlobalSMData("hat_state", static_cast<int>(GuestAppearance::UNKNOWN));
    getStateMachine()->setGlobalSMData("hat_confidence", 0.0);
    getStateMachine()->setGlobalSMData(
      "face_mask_state",
      static_cast<int>(GuestAppearance::UNKNOWN));
    getStateMachine()->setGlobalSMData("face_mask_confidence", 0.0);
    getStateMachine()->setGlobalSMData("feature_summary_text", std::string{});

    RCLCPP_INFO(getLogger(), "Cleared temporary guest information and detected features.");
    return memory_cleared;
  }

  std::string step_name_;
};

// Section: loop-control behavior that repeats steps 1-10 once, then finishes.
class CbCheckLoop : public smacc2::SmaccAsyncClientBehavior
{
public:
  // Special function: decide whether to run another full Task Receptionist cycle.
  void onEntry() override
  {
    int cycle_index = 1;
    int max_cycles = 2;
    getStateMachine()->getGlobalSMData("cycle_index", cycle_index);
    getStateMachine()->getGlobalSMData("max_cycles", max_cycles);

    if (cycle_index < max_cycles) {
      const int next_cycle = cycle_index + 1;
      getStateMachine()->setGlobalSMData("cycle_index", next_cycle);
      RCLCPP_INFO(getLogger(), "Starting Task Receptionist cycle %d of %d.", next_cycle, max_cycles);
      postEvent<events::EvLoopAgain>();
      return;
    }

    RCLCPP_INFO(getLogger(), "Task Receptionist completed %d cycle(s).", max_cycles);
    postEvent<events::EvTaskComplete>();
  }
};
}  // namespace client_behaviors

// Special helper: request SMACC2 stop and ROS shutdown outside the active state callback stack.
void requestSmaccStopAndShutdown(smacc2::ISmaccStateMachine * state_machine, int exit_code)
{
  std::thread(
    [state_machine, exit_code]() {
      bool force_process_exit = true;
      if (state_machine != nullptr) {
        const auto node = state_machine->getNode();
        force_process_exit = node->get_parameter("force_process_exit_after_terminal_state").as_bool();
      }

      rclcpp::sleep_for(std::chrono::milliseconds(100));
      process_utils::stopRegisteredInsightFace(rclcpp::get_logger("task_receptionist_shutdown"));
      if (state_machine != nullptr) {
        state_machine->stop();
      }

      rclcpp::sleep_for(std::chrono::milliseconds(100));
      if (force_process_exit) {
        std::fflush(nullptr);
        std::_Exit(exit_code);
      }
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    })
    .detach();
}

// Section: reusable linear state template for exact-flow success/failure transitions.
template<typename StateT, typename NextStateT>
struct LinearTaskState : smacc2::SmaccState<StateT, SmTaskReceptionist>
{
  using SmaccState = smacc2::SmaccState<StateT, SmTaskReceptionist>;
  using StepSuccess =
    smacc2::EvCbSuccess<client_behaviors::CbTaskReceptionistStep, orthogonals::OrTask>;
  using StepFailure =
    smacc2::EvCbFailure<client_behaviors::CbTaskReceptionistStep, orthogonals::OrTask>;
  using reactions = boost::mpl::list<
    smacc2::Transition<StepSuccess, NextStateT, smacc2::SUCCESS>,
    smacc2::Transition<StepFailure, StAborted, smacc2::ABORT>>;
  using SmaccState::SmaccState;
};

struct StStartupReadiness : LinearTaskState<StStartupReadiness, StWaitBeforeGuest>
{
  using LinearTaskState<StStartupReadiness, StWaitBeforeGuest>::LinearTaskState;

  // Special function: bind startup dependency readiness checks to this first state.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "startup_readiness");
  }
};

struct StWaitBeforeGuest : LinearTaskState<StWaitBeforeGuest, StNavigateToWaypoint7>
{
  using LinearTaskState<StWaitBeforeGuest, StNavigateToWaypoint7>::LinearTaskState;

  // Special function: bind step 1 wait behavior to this state.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "wait_before_guest");
  }
};

struct StNavigateToWaypoint7 : LinearTaskState<StNavigateToWaypoint7, StLaunchInsightFace>
{
  using LinearTaskState<StNavigateToWaypoint7, StLaunchInsightFace>::LinearTaskState;

  // Special function: bind step 1 navigation to waypoint 7.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "navigate_waypoint_7");
  }
};

struct StLaunchInsightFace : LinearTaskState<StLaunchInsightFace, StWaitForFace>
{
  using LinearTaskState<StLaunchInsightFace, StWaitForFace>::LinearTaskState;

  // Special function: bind step 2 InsightFace activation.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "launch_insightface");
  }
};

struct StWaitForFace : LinearTaskState<StWaitForFace, StCollectAppearance>
{
  using LinearTaskState<StWaitForFace, StCollectAppearance>::LinearTaskState;

  // Special function: bind step 2 face-detection wait.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "wait_for_face");
  }
};

struct StCollectAppearance : LinearTaskState<StCollectAppearance, StWaitAfterFeatures>
{
  using LinearTaskState<StCollectAppearance, StWaitAfterFeatures>::LinearTaskState;

  // Special function: bind step 2 feature recording.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "collect_appearance");
  }
};

struct StWaitAfterFeatures : LinearTaskState<StWaitAfterFeatures, StAskGuestInformation>
{
  using LinearTaskState<StWaitAfterFeatures, StAskGuestInformation>::LinearTaskState;

  // Special function: bind the requested 2-second wait after feature recording.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "wait_after_features");
  }
};

struct StAskGuestInformation : LinearTaskState<StAskGuestInformation, StSayFollowMe>
{
  using LinearTaskState<StAskGuestInformation, StSayFollowMe>::LinearTaskState;

  // Special function: bind step 3 dialogue action goal.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "ask_guest_information");
  }
};

struct StSayFollowMe : LinearTaskState<StSayFollowMe, StNavigateToWaypoint6>
{
  using LinearTaskState<StSayFollowMe, StNavigateToWaypoint6>::LinearTaskState;

  // Special function: bind step 4 follow-me speech.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "say_follow_me");
  }
};

struct StNavigateToWaypoint6 : LinearTaskState<StNavigateToWaypoint6, StSpeakFeatureSummary>
{
  using LinearTaskState<StNavigateToWaypoint6, StSpeakFeatureSummary>::LinearTaskState;

  // Special function: bind step 5 navigation to waypoint 6.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "navigate_waypoint_6");
  }
};

struct StSpeakFeatureSummary : LinearTaskState<StSpeakFeatureSummary, StSpeakGuestInformation>
{
  using LinearTaskState<StSpeakFeatureSummary, StSpeakGuestInformation>::LinearTaskState;

  // Special function: bind step 6 InsightFace feature speech.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "speak_feature_summary");
  }
};

struct StSpeakGuestInformation : LinearTaskState<StSpeakGuestInformation, StNavigateToWaypoint8>
{
  using LinearTaskState<StSpeakGuestInformation, StNavigateToWaypoint8>::LinearTaskState;

  // Special function: bind step 7 guest information speech.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "speak_guest_information");
  }
};

struct StNavigateToWaypoint8 : LinearTaskState<StNavigateToWaypoint8, StSaySeatMiddle>
{
  using LinearTaskState<StNavigateToWaypoint8, StSaySeatMiddle>::LinearTaskState;

  // Special function: bind step 8 navigation to waypoint 8.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "navigate_waypoint_8");
  }
};

struct StSaySeatMiddle : LinearTaskState<StSaySeatMiddle, StNavigateToWaypoint99>
{
  using LinearTaskState<StSaySeatMiddle, StNavigateToWaypoint99>::LinearTaskState;

  // Special function: bind step 9 seating instruction speech.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "say_seat_middle");
  }
};

struct StNavigateToWaypoint99 : LinearTaskState<StNavigateToWaypoint99, StClearGuestData>
{
  using LinearTaskState<StNavigateToWaypoint99, StClearGuestData>::LinearTaskState;

  // Special function: bind step 10 navigation to waypoint 99.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "navigate_waypoint_99");
  }
};

struct StClearGuestData : LinearTaskState<StClearGuestData, StCheckLoop>
{
  using LinearTaskState<StClearGuestData, StCheckLoop>::LinearTaskState;

  // Special function: bind step 11 cleanup of guest info and detected features.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbTaskReceptionistStep>(
      "clear_guest_data");
  }
};

struct StCheckLoop : smacc2::SmaccState<StCheckLoop, SmTaskReceptionist>
{
  using SmaccState::SmaccState;
  using reactions = boost::mpl::list<
    smacc2::Transition<events::EvLoopAgain, StWaitBeforeGuest, smacc2::SUCCESS>,
    smacc2::Transition<events::EvTaskComplete, StFinished, smacc2::SUCCESS>>;

  // Special function: bind the loop check that repeats steps 1-10 exactly once.
  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbCheckLoop>();
  }
};

struct StFinished : smacc2::SmaccState<StFinished, SmTaskReceptionist>
{
  using SmaccState::SmaccState;

  // Special function: terminal success state has no client behavior.
  static void staticConfigure() {}

  // Special function: stop background processes and finish the SMACC2 child with success.
  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "Task Receptionist finished successfully.");
    requestSmaccStopAndShutdown(&getStateMachine(), 0);
  }
};

struct StAborted : smacc2::SmaccState<StAborted, SmTaskReceptionist>
{
  using SmaccState::SmaccState;

  // Special function: terminal abort state has no client behavior.
  static void staticConfigure() {}

  // Special function: stop background processes and finish the SMACC2 child with failure.
  void onEntry()
  {
    std::string last_error;
    getStateMachine().getGlobalSMData("last_error", last_error);
    RCLCPP_ERROR(
      getLogger(),
      "Task Receptionist aborted safely. Last error: %s",
      last_error.empty() ? "unspecified" : last_error.c_str());
    requestSmaccStopAndShutdown(&getStateMachine(), 1);
  }
};
}  // namespace sm_receptionist::task_receptionist

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  smacc2::run<sm_receptionist::task_receptionist::SmTaskReceptionist>();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}


/*TODO 
   1. Don't map all fail cases to StAborted
   2. Make sure lifecycle node do not turn off my_robot_nav 
   3. Check why /perception/largest_face (detected=False) at the first attempt.
*/
