#include <boost/mpl/list.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdint>
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
#include <receptionist_interfaces/msg/guest_appearance.hpp>
#include <smacc2/smacc.hpp>
#include <std_msgs/msg/string.hpp>
#include <voice_interfaces/srv/say_text.hpp>

#include "insightface_feature_summary_smacc2/process_utils.hpp"

namespace insightface_feature_summary_smacc2
{
namespace orthogonals
{
// Section: single task orthogonal used by all sequential client behaviors.
struct OrTask : smacc2::Orthogonal<OrTask>
{
};
}  // namespace orthogonals

// Section: forward declarations for SMACC2 states.
struct StLaunchInsightFace;
struct StWaitForFace;
struct StCollectAppearance;
struct StBuildSummary;
struct StSpeakSummary;
struct StStopInsightFaceAndFinish;
struct StAborted;

// Section: state machine definition and shared global-data initialization.
struct SmInsightFaceFeatureSummary
: smacc2::SmaccStateMachineBase<SmInsightFaceFeatureSummary, StLaunchInsightFace>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  // Special function: declare ROS parameters and copy them into SMACC2 global data.
  void onInitialize() override
  {
    auto node = getNode();

    node->declare_parameter<bool>("personal_test_mode", true);
    node->declare_parameter<std::string>("camera_index", "0");
    node->declare_parameter<std::string>("camera_backend", "realsense");
    node->declare_parameter<std::string>("image_topic", "/camera/camera/color/image_raw");
    node->declare_parameter<bool>("enable_depth", true);
    node->declare_parameter<std::string>("face_topic", "/perception/largest_face");
    node->declare_parameter<std::string>("appearance_topic", "/appearance/current_estimate");
    node->declare_parameter<std::string>("tts_service_name", "/voice/tts/say");
    node->declare_parameter<double>("face_wait_timeout_sec", 10.0);
    node->declare_parameter<double>("appearance_wait_timeout_sec", 10.0);
    node->declare_parameter<double>("collection_duration_sec", 2.0);
    node->declare_parameter<int>("min_stable_samples", 3);
    node->declare_parameter<bool>("show_appearance_gui", true);
    node->declare_parameter<double>("stream_info_period_sec", 1.0);

    setGlobalSMData("personal_test_mode", node->get_parameter("personal_test_mode").as_bool());
    setGlobalSMData("camera_index", node->get_parameter("camera_index").as_string());
    setGlobalSMData("camera_backend", node->get_parameter("camera_backend").as_string());
    setGlobalSMData("image_topic", node->get_parameter("image_topic").as_string());
    setGlobalSMData("enable_depth", node->get_parameter("enable_depth").as_bool());
    setGlobalSMData("face_topic", node->get_parameter("face_topic").as_string());
    setGlobalSMData("appearance_topic", node->get_parameter("appearance_topic").as_string());
    setGlobalSMData("tts_service_name", node->get_parameter("tts_service_name").as_string());
    setGlobalSMData("face_wait_timeout_sec", node->get_parameter("face_wait_timeout_sec").as_double());
    setGlobalSMData(
      "appearance_wait_timeout_sec",
      node->get_parameter("appearance_wait_timeout_sec").as_double());
    setGlobalSMData(
      "collection_duration_sec",
      node->get_parameter("collection_duration_sec").as_double());
    setGlobalSMData("min_stable_samples", node->get_parameter("min_stable_samples").as_int());
    setGlobalSMData("show_appearance_gui", node->get_parameter("show_appearance_gui").as_bool());
    setGlobalSMData(
      "stream_info_period_sec",
      node->get_parameter("stream_info_period_sec").as_double());

    setGlobalSMData("active_insightface_pgid", static_cast<std::int64_t>(-1));
    setGlobalSMData("summary_sample_count", 0);
    setGlobalSMData("shirt_color", std::string{});
    setGlobalSMData("shirt_color_confidence", 0.0);
    setGlobalSMData(
      "glasses_state",
      static_cast<int>(receptionist_interfaces::msg::GuestAppearance::UNKNOWN));
    setGlobalSMData("glasses_confidence", 0.0);
    setGlobalSMData(
      "hat_state",
      static_cast<int>(receptionist_interfaces::msg::GuestAppearance::UNKNOWN));
    setGlobalSMData("hat_confidence", 0.0);
    setGlobalSMData(
      "face_mask_state",
      static_cast<int>(receptionist_interfaces::msg::GuestAppearance::UNKNOWN));
    setGlobalSMData("face_mask_confidence", 0.0);
    setGlobalSMData("feature_summary_text", std::string{});

    process_utils::clearInsightFacePgid();
    createOrthogonal<orthogonals::OrTask>();

    RCLCPP_INFO(
      getLogger(),
      "Feature summary SMACC2 initialized: personal_test_mode=%s camera_index=%s "
      "face_topic=%s appearance_topic=%s tts_service=%s",
      node->get_parameter("personal_test_mode").as_bool() ? "true" : "false",
      node->get_parameter("camera_index").as_string().c_str(),
      node->get_parameter("face_topic").as_string().c_str(),
      node->get_parameter("appearance_topic").as_string().c_str(),
      node->get_parameter("tts_service_name").as_string().c_str());
  }
};

namespace client_behaviors
{
using GuestAppearance = receptionist_interfaces::msg::GuestAppearance;
using SayText = voice_interfaces::srv::SayText;

// Section: formatting helpers for launch arguments and generated speech.
std::string boolString(bool value)
{
  return value ? "true" : "false";
}

// Special helper: render numeric launch arguments with stable precision.
std::string formatDouble(double value)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << value;
  return out.str();
}

// Special helper: build one ROS launch argument as a safely quoted shell token.
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

// Special helper: accumulate one YES/NO/UNKNOWN attribute sample.
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

// Special helper: convert attribute votes into a single stable state.
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
    if (score.second > best_color_count || (score.second == best_color_count &&
      average > best_color_average))
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

// Special helper: create the final flexible TTS text from global appearance data.
std::string buildFeatureSummaryText(
  const std::string & shirt_color,
  int glasses_state,
  int hat_state,
  int face_mask_state)
{
  std::ostringstream text;

  if (!shirt_color.empty()) {
    text << "I can see a person wearing a " << shirt_color << " shirt. ";
  } else {
    text << "I can see a person, but I cannot confidently determine the shirt color. ";
  }

  text << describeAttribute(
    glasses_state,
    "I detect glasses. ",
    "I do not detect glasses. ",
    "I cannot confidently determine whether the person is wearing glasses. ");
  text << describeAttribute(
    hat_state,
    "I detect a hat. ",
    "I do not detect a hat. ",
    "I cannot confidently determine whether the person is wearing a hat. ");
  text << describeAttribute(
    face_mask_state,
    "I detect a face mask.",
    "I do not detect a face mask.",
    "I cannot confidently determine whether the person is wearing a face mask.");

  return text.str();
}

// Section: launch InsightFace appearance pipeline.
class CbLaunchInsightFace : public smacc2::SmaccAsyncClientBehavior
{
public:
  // Special function: start the InsightFace launch tree and register its process group.
  void onEntry() override
  {
    bool personal_test_mode = true;
    std::string camera_index = "0";
    std::string camera_backend = "realsense";
    std::string image_topic = "/camera/camera/color/image_raw";
    bool enable_depth = true;
    std::string face_topic = "/perception/largest_face";
    std::string appearance_topic = "/appearance/current_estimate";
    bool show_appearance_gui = true;
    double stream_info_period_sec = 1.0;

    getStateMachine()->getGlobalSMData("personal_test_mode", personal_test_mode);
    getStateMachine()->getGlobalSMData("camera_index", camera_index);
    getStateMachine()->getGlobalSMData("camera_backend", camera_backend);
    getStateMachine()->getGlobalSMData("image_topic", image_topic);
    getStateMachine()->getGlobalSMData("enable_depth", enable_depth);
    getStateMachine()->getGlobalSMData("face_topic", face_topic);
    getStateMachine()->getGlobalSMData("appearance_topic", appearance_topic);
    getStateMachine()->getGlobalSMData("show_appearance_gui", show_appearance_gui);
    getStateMachine()->getGlobalSMData("stream_info_period_sec", stream_info_period_sec);

    process_utils::stopRegisteredInsightFace(getLogger());

    const auto command = buildInsightFaceLaunchCommand(
      personal_test_mode,
      camera_index,
      camera_backend,
      image_topic,
      enable_depth,
      face_topic,
      appearance_topic,
      show_appearance_gui,
      stream_info_period_sec);
    const auto pid = process_utils::launchDetachedProcess(command);
    if (pid <= 0) {
      RCLCPP_ERROR(getLogger(), "Failed to launch InsightFace appearance pipeline.");
      postFailureEvent();
      return;
    }

    process_utils::writeInsightFacePgid(pid);
    getStateMachine()->setGlobalSMData(
      "active_insightface_pgid",
      static_cast<std::int64_t>(pid));

    RCLCPP_INFO(
      getLogger(),
      "Started InsightFace feature pipeline with process group %d. Command: %s",
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
};

// Section: wait until face_perception confirms a visible face.
class CbWaitForFace : public smacc2::SmaccAsyncClientBehavior
{
public:
  // Special function: subscribe to the face JSON topic and wait for detected=true.
  void onEntry() override
  {
    std::string face_topic = "/perception/largest_face";
    double timeout_sec = 10.0;
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
      "Waiting for face detection on %s with timeout %.2f seconds.",
      face_topic.c_str(),
      timeout_sec);

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !isShutdownRequested() &&
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

// Section: collect several GuestAppearance messages and store stable features.
class CbCollectAppearance : public smacc2::SmaccAsyncClientBehavior
{
public:
  // Special function: collect appearance samples, aggregate them, and save global data.
  void onEntry() override
  {
    std::string appearance_topic = "/appearance/current_estimate";
    double timeout_sec = 10.0;
    double collection_duration_sec = 2.0;
    int min_stable_samples = 3;
    getStateMachine()->getGlobalSMData("appearance_topic", appearance_topic);
    getStateMachine()->getGlobalSMData("appearance_wait_timeout_sec", timeout_sec);
    getStateMachine()->getGlobalSMData("collection_duration_sec", collection_duration_sec);
    getStateMachine()->getGlobalSMData("min_stable_samples", min_stable_samples);

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
      "Collecting appearance samples on %s for %.2f seconds, target samples=%d, timeout=%.2f.",
      appearance_topic.c_str(),
      collection_duration_sec,
      min_stable_samples,
      timeout_sec);

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !isShutdownRequested()) {
      std::int64_t pgid = -1;
      getStateMachine()->getGlobalSMData("active_insightface_pgid", pgid);
      if (pgid > 0 && !process_utils::processGroupRunning(static_cast<pid_t>(pgid))) {
        RCLCPP_ERROR(
          getLogger(),
          "InsightFace process group %ld exited during appearance collection.",
          static_cast<long>(pgid));
        postFailureEvent();
        return;
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
          collection_window_elapsed = elapsed_collection >= std::max(0.0, collection_duration_sec);
        }
      }

      if (enough_samples && collection_window_elapsed) {
        break;
      }

      const auto elapsed_total = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      if (timeout_sec > 0.0 && elapsed_total >= timeout_sec) {
        if (sample_count <= 0) {
          RCLCPP_ERROR(
            getLogger(),
            "Timed out after %.2f seconds waiting for appearance samples on %s.",
            timeout_sec,
            appearance_topic.c_str());
          postFailureEvent();
          return;
        }
        RCLCPP_WARN(
          getLogger(),
          "Appearance collection timed out with %d sample(s); using available samples.",
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
      postFailureEvent();
      return;
    }

    const auto aggregate = aggregateAppearance(snapshot);
    getStateMachine()->setGlobalSMData("summary_sample_count", aggregate.sample_count);
    getStateMachine()->setGlobalSMData("shirt_color", aggregate.shirt_color);
    getStateMachine()->setGlobalSMData("shirt_color_confidence", aggregate.shirt_color_confidence);
    getStateMachine()->setGlobalSMData("glasses_state", aggregate.glasses_state);
    getStateMachine()->setGlobalSMData("glasses_confidence", aggregate.glasses_confidence);
    getStateMachine()->setGlobalSMData("hat_state", aggregate.hat_state);
    getStateMachine()->setGlobalSMData("hat_confidence", aggregate.hat_confidence);
    getStateMachine()->setGlobalSMData("face_mask_state", aggregate.face_mask_state);
    getStateMachine()->setGlobalSMData("face_mask_confidence", aggregate.face_mask_confidence);

    RCLCPP_INFO(
      getLogger(),
      "Appearance summary stored from %d samples: shirt=%s glasses=%d hat=%d face_mask=%d.",
      aggregate.sample_count,
      aggregate.shirt_color.empty() ? "unknown" : aggregate.shirt_color.c_str(),
      aggregate.glasses_state,
      aggregate.hat_state,
      aggregate.face_mask_state);
    postSuccessEvent();
  }
};

// Section: generate flexible speech text from the stored appearance features.
class CbBuildFeatureSummary : public smacc2::SmaccAsyncClientBehavior
{
public:
  // Special function: read feature global data and write the final TTS text global data.
  void onEntry() override
  {
    std::string shirt_color;
    int glasses_state = static_cast<int>(GuestAppearance::UNKNOWN);
    int hat_state = static_cast<int>(GuestAppearance::UNKNOWN);
    int face_mask_state = static_cast<int>(GuestAppearance::UNKNOWN);

    getStateMachine()->getGlobalSMData("shirt_color", shirt_color);
    getStateMachine()->getGlobalSMData("glasses_state", glasses_state);
    getStateMachine()->getGlobalSMData("hat_state", hat_state);
    getStateMachine()->getGlobalSMData("face_mask_state", face_mask_state);

    const auto summary_text = buildFeatureSummaryText(
      shirt_color,
      glasses_state,
      hat_state,
      face_mask_state);
    getStateMachine()->setGlobalSMData("feature_summary_text", summary_text);

    RCLCPP_INFO(getLogger(), "Generated feature summary text: %s", summary_text.c_str());
    postSuccessEvent();
  }
};

// Section: call the TTS service exactly once with the generated summary text.
class CbSpeakFeatureSummary : public smacc2::SmaccAsyncClientBehavior
{
public:
  // Special function: send one SayText request to the configured TTS service.
  void onEntry() override
  {
    using namespace std::chrono_literals;

    std::string service_name = "/voice/tts/say";
    std::string text;
    getStateMachine()->getGlobalSMData("tts_service_name", service_name);
    getStateMachine()->getGlobalSMData("feature_summary_text", text);

    if (text.empty()) {
      RCLCPP_ERROR(getLogger(), "No feature summary text is available for TTS.");
      postFailureEvent();
      return;
    }

    auto node = getNode();
    auto client = node->create_client<SayText>(service_name);
    const auto service_timeout = 10s;
    if (!client->wait_for_service(service_timeout)) {
      RCLCPP_ERROR(getLogger(), "%s service is not available.", service_name.c_str());
      postFailureEvent();
      return;
    }

    auto request = std::make_shared<SayText::Request>();
    request->text = text;
    auto future = client->async_send_request(request);

    const auto response_timeout = 30s;
    const auto start = std::chrono::steady_clock::now();
    while (!isShutdownRequested() && rclcpp::ok() &&
      future.wait_for(100ms) != std::future_status::ready)
    {
      if ((std::chrono::steady_clock::now() - start) > response_timeout) {
        RCLCPP_ERROR(getLogger(), "%s service call timed out.", service_name.c_str());
        postFailureEvent();
        return;
      }
      rclcpp::sleep_for(100ms);
    }

    if (!rclcpp::ok() || isShutdownRequested()) {
      postFailureEvent();
      return;
    }

    auto response = future.get();
    if (response == nullptr || !response->success) {
      RCLCPP_ERROR(
        getLogger(),
        "%s failed: %s",
        service_name.c_str(),
        response ? response->message.c_str() : "no response");
      postFailureEvent();
      return;
    }

    RCLCPP_INFO(getLogger(), "TTS completed: %s", response->message.c_str());
    postSuccessEvent();
  }
};
}  // namespace client_behaviors

// Special helper: request shutdown from outside the current state callback stack.
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

// Section: SMACC2 state that launches the InsightFace pipeline.
struct StLaunchInsightFace : smacc2::SmaccState<StLaunchInsightFace, SmInsightFaceFeatureSummary>
{
  using SmaccState::SmaccState;
  using LaunchSuccess = smacc2::EvCbSuccess<
    client_behaviors::CbLaunchInsightFace,
    orthogonals::OrTask>;
  using LaunchFailure = smacc2::EvCbFailure<
    client_behaviors::CbLaunchInsightFace,
    orthogonals::OrTask>;

  using reactions = boost::mpl::list<
    smacc2::Transition<LaunchSuccess, StWaitForFace, smacc2::SUCCESS>,
    smacc2::Transition<LaunchFailure, StAborted, smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbLaunchInsightFace>();
  }
};

// Section: SMACC2 state that waits for a face before collecting appearance.
struct StWaitForFace : smacc2::SmaccState<StWaitForFace, SmInsightFaceFeatureSummary>
{
  using SmaccState::SmaccState;
  using WaitSuccess = smacc2::EvCbSuccess<client_behaviors::CbWaitForFace, orthogonals::OrTask>;
  using WaitFailure = smacc2::EvCbFailure<client_behaviors::CbWaitForFace, orthogonals::OrTask>;

  using reactions = boost::mpl::list<
    smacc2::Transition<WaitSuccess, StCollectAppearance, smacc2::SUCCESS>,
    smacc2::Transition<WaitFailure, StAborted, smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbWaitForFace>();
  }
};

// Section: SMACC2 state that stores the detected feature data.
struct StCollectAppearance : smacc2::SmaccState<StCollectAppearance, SmInsightFaceFeatureSummary>
{
  using SmaccState::SmaccState;
  using CollectSuccess = smacc2::EvCbSuccess<
    client_behaviors::CbCollectAppearance,
    orthogonals::OrTask>;
  using CollectFailure = smacc2::EvCbFailure<
    client_behaviors::CbCollectAppearance,
    orthogonals::OrTask>;

  using reactions = boost::mpl::list<
    smacc2::Transition<CollectSuccess, StBuildSummary, smacc2::SUCCESS>,
    smacc2::Transition<CollectFailure, StAborted, smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbCollectAppearance>();
  }
};

// Section: SMACC2 state that converts stored features into flexible TTS text.
struct StBuildSummary : smacc2::SmaccState<StBuildSummary, SmInsightFaceFeatureSummary>
{
  using SmaccState::SmaccState;
  using BuildSuccess = smacc2::EvCbSuccess<
    client_behaviors::CbBuildFeatureSummary,
    orthogonals::OrTask>;
  using BuildFailure = smacc2::EvCbFailure<
    client_behaviors::CbBuildFeatureSummary,
    orthogonals::OrTask>;

  using reactions = boost::mpl::list<
    smacc2::Transition<BuildSuccess, StSpeakSummary, smacc2::SUCCESS>,
    smacc2::Transition<BuildFailure, StAborted, smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbBuildFeatureSummary>();
  }
};

// Section: SMACC2 state that performs the one and only TTS service call.
struct StSpeakSummary : smacc2::SmaccState<StSpeakSummary, SmInsightFaceFeatureSummary>
{
  using SmaccState::SmaccState;
  using SpeakSuccess = smacc2::EvCbSuccess<
    client_behaviors::CbSpeakFeatureSummary,
    orthogonals::OrTask>;
  using SpeakFailure = smacc2::EvCbFailure<
    client_behaviors::CbSpeakFeatureSummary,
    orthogonals::OrTask>;

  using reactions = boost::mpl::list<
    smacc2::Transition<SpeakSuccess, StStopInsightFaceAndFinish, smacc2::SUCCESS>,
    smacc2::Transition<SpeakFailure, StAborted, smacc2::ABORT>>;

  static void staticConfigure()
  {
    configure_orthogonal<orthogonals::OrTask, client_behaviors::CbSpeakFeatureSummary>();
  }
};

// Section: terminal success state that cleans up InsightFace before shutdown.
struct StStopInsightFaceAndFinish
: smacc2::SmaccState<StStopInsightFaceAndFinish, SmInsightFaceFeatureSummary>
{
  using SmaccState::SmaccState;

  static void staticConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "Feature summary completed. Stopping InsightFace.");
    process_utils::stopRegisteredInsightFace(getLogger());
    getStateMachine().setGlobalSMData("active_insightface_pgid", static_cast<std::int64_t>(-1));
    requestSmaccShutdown();
  }
};

// Section: terminal failure state that still cleans up InsightFace before shutdown.
struct StAborted : smacc2::SmaccState<StAborted, SmInsightFaceFeatureSummary>
{
  using SmaccState::SmaccState;

  static void staticConfigure() {}

  void onEntry()
  {
    RCLCPP_ERROR(getLogger(), "Feature summary state machine aborted. Cleaning InsightFace.");
    process_utils::stopRegisteredInsightFace(getLogger());
    getStateMachine().setGlobalSMData("active_insightface_pgid", static_cast<std::int64_t>(-1));
    requestSmaccShutdown();
  }
};
}  // namespace insightface_feature_summary_smacc2

namespace
{
// Special helper: safety cleanup if ROS shutdown happens outside the terminal states.
void forceStopInsightFaceOnShutdown()
{
  const auto logger = rclcpp::get_logger("feature_summary_smacc2_shutdown");
  insightface_feature_summary_smacc2::process_utils::stopRegisteredInsightFace(logger);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::on_shutdown(forceStopInsightFaceOnShutdown);
  smacc2::run<insightface_feature_summary_smacc2::SmInsightFaceFeatureSummary>();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
