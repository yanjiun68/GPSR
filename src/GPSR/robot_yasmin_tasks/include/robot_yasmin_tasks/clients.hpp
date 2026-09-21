#ifndef ROBOT_YASMIN_TASKS__CLIENTS_HPP_
#define ROBOT_YASMIN_TASKS__CLIENTS_HPP_

#include "robot_yasmin_tasks/runtime.hpp"

// Section: ROS interface types used by deterministic client adapters.
#include <condition_variable>
#include <mutex>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <putrabot_interfaces/action/person_follow.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <receptionist_interfaces/action/navigate_named_pose.hpp>
#include <robot_ai_interfaces/action/count_objects.hpp>
#include <robot_ai_interfaces/action/count_people.hpp>
#include <robot_ai_interfaces/action/extract_guest_info.hpp>
#include <robot_ai_interfaces/action/find_object.hpp>
#include <robot_ai_interfaces/action/find_person_by_attribute.hpp>
#include <robot_ai_interfaces/action/guide_person.hpp>
#include <robot_ai_interfaces/action/handover.hpp>
#include <robot_ai_interfaces/action/pick_object.hpp>
#include <robot_ai_interfaces/action/place_object.hpp>
#include <robot_ai_interfaces/srv/manage_lifecycle_skill.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <voice_interfaces/srv/say_text.hpp>

namespace robot_yasmin_tasks {

// Section: Common callable signature used by SimpleSkillSM.
using SkillClientCall =
    std::function<SkillResultData(const Blackboard::SharedPtr &,
                                  const ActionStepData &)>;

class SpeechClient {
 public:
  // Special function: Create the persistent TTS service client.
  explicit SpeechClient(const rclcpp::Node::SharedPtr &node,
                        bool simulate = false,
                        std::string service_name = "/voice/tts/say");

  // Special function: Render deferred variables and invoke the TTS service.
  SkillResultData say(const Blackboard::SharedPtr &blackboard,
                      const ActionStepData &action);

 private:
  rclcpp::Node::SharedPtr node_;
  bool simulate_;
  std::string service_name_;
  rclcpp::Client<voice_interfaces::srv::SayText>::SharedPtr client_;
};

class DialogueClient {
 public:
  // Special function: Create persistent TTS and STT topic interfaces.
  DialogueClient(const rclcpp::Node::SharedPtr &node,
                 const std::shared_ptr<SpeechClient> &speech,
                 bool simulate = false);

  // Special function: Ask one question and wait for the next transcript.
  SkillResultData ask_and_listen(const Blackboard::SharedPtr &blackboard,
                                 const ActionStepData &action);

 private:
  // Special function: Cache one transcript and wake the active dialogue wait.
  void transcript_callback(const std_msgs::msg::String::SharedPtr message);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<SpeechClient> speech_;
  bool simulate_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::string latest_;
  std::uint64_t sequence_{0};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr start_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr transcript_sub_;
};

class Nav2Client {
 public:
  // Special function: Create named-pose, raw-pose, and spin action clients.
  explicit Nav2Client(
      const rclcpp::Node::SharedPtr &node, bool simulate = false,
      std::string named_action = "/nav/navigate_named_pose",
      std::string pose_action = "/navigate_to_pose",
      std::string spin_action = "/spin");

  // Special function: Route mutually exclusive named and raw-pose navigation.
  SkillResultData navigate_to_pose(const Blackboard::SharedPtr &blackboard,
                                   const ActionStepData &action);

  // Special function: Rotate through one relative yaw for person search.
  SkillResultData spin(const Blackboard::SharedPtr &blackboard,
                       const ActionStepData &action, double yaw_radians,
                       double timeout_sec);

 private:
  rclcpp::Node::SharedPtr node_;
  bool simulate_;
  std::string named_action_;
  std::string pose_action_;
  std::string spin_action_;
  rclcpp_action::Client<receptionist_interfaces::action::NavigateNamedPose>::SharedPtr
      named_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr pose_client_;
  rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client_;
};

class PerceptionClient {
 public:
  // Special function: Create the two GPSR count action clients.
  explicit PerceptionClient(
      const rclcpp::Node::SharedPtr &node, bool simulate = false,
      std::string object_action = "/gpsr/count_objects",
      std::string people_action = "/gpsr/count_people");

  // Special function: Execute object counting and update its blackboard contract.
  SkillResultData count_objects(const Blackboard::SharedPtr &blackboard,
                                const ActionStepData &action);

  // Special function: Execute people counting and update its blackboard contract.
  SkillResultData count_people(const Blackboard::SharedPtr &blackboard,
                               const ActionStepData &action);

  // Special function: Preserve the local object comparison behavior.
  SkillResultData compare_objects(const Blackboard::SharedPtr &blackboard,
                                  const ActionStepData &action);

  // Special function: Preserve the local person-information extraction fallback.
  SkillResultData extract_person_info(const Blackboard::SharedPtr &blackboard,
                                      const ActionStepData &action);

 private:
  rclcpp::Node::SharedPtr node_;
  bool simulate_;
  std::string object_action_;
  std::string people_action_;
  rclcpp_action::Client<robot_ai_interfaces::action::CountObjects>::SharedPtr
      object_client_;
  rclcpp_action::Client<robot_ai_interfaces::action::CountPeople>::SharedPtr
      people_client_;
};

class FollowPersonClient {
 public:
  // Special function: Create the production PersonFollow action client.
  explicit FollowPersonClient(
      const rclcpp::Node::SharedPtr &node,
      std::string action_name = "/person_follow/master");

  // Special function: Forward one FOLLOW_PERSON action with cancellation and timeout.
  SkillResultData follow(const Blackboard::SharedPtr &blackboard,
                         const ActionStepData &action);

 private:
  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  rclcpp_action::Client<putrabot_interfaces::action::PersonFollow>::SharedPtr
      client_;
};

class RemoteSkillClient {
 public:
  // Special function: Create persistent clients for lifecycle-owned compound skills.
  explicit RemoteSkillClient(const rclcpp::Node::SharedPtr &node);

  // Special function: Execute the lifecycle-owned FIND_OBJECT mission.
  SkillResultData find_object(const Blackboard::SharedPtr &blackboard,
                              const ActionStepData &action);

  // Special function: Execute the lifecycle-owned attribute-person mission.
  SkillResultData find_person_by_attribute(
      const Blackboard::SharedPtr &blackboard, const ActionStepData &action);

  // Special function: Execute lifecycle-owned guest-information extraction.
  SkillResultData extract_guest_info(const Blackboard::SharedPtr &blackboard,
                                     const ActionStepData &action);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<robot_ai_interfaces::action::FindObject>::SharedPtr
      find_object_client_;
  rclcpp_action::Client<
      robot_ai_interfaces::action::FindPersonByAttribute>::SharedPtr
      find_attribute_client_;
  rclcpp_action::Client<robot_ai_interfaces::action::ExtractGuestInfo>::SharedPtr
      extract_client_;
};

class ManipulationClient {
 public:
  // Special function: Create the three production manipulation action clients.
  explicit ManipulationClient(
      const rclcpp::Node::SharedPtr &node = nullptr, bool simulate = false,
      std::string pick_action = "/manipulation/pick_object",
      std::string place_action = "/manipulation/place_object",
      std::string handover_action = "/manipulation/handover_object");

  // Section: Execute manipulation goals and maintain the shared held-object contract.
  SkillResultData pick_object(const Blackboard::SharedPtr &blackboard,
                              const ActionStepData &action);
  SkillResultData place_object(const Blackboard::SharedPtr &blackboard,
                               const ActionStepData &action);
  SkillResultData dispose_trash(const Blackboard::SharedPtr &blackboard,
                                const ActionStepData &action);
  SkillResultData handover(const Blackboard::SharedPtr &blackboard,
                           const ActionStepData &action);

 private:
  rclcpp::Node::SharedPtr node_;
  bool simulate_;
  rclcpp_action::Client<robot_ai_interfaces::action::PickObject>::SharedPtr
      pick_client_;
  rclcpp_action::Client<robot_ai_interfaces::action::PlaceObject>::SharedPtr
      place_client_;
  rclcpp_action::Client<robot_ai_interfaces::action::Handover>::SharedPtr
      handover_client_;
};

class GuidePersonClient {
 public:
  // Special function: Create the navigation-motion guide action client.
  explicit GuidePersonClient(
      const rclcpp::Node::SharedPtr &node = nullptr, bool simulate = false,
      std::string action_name = "/person_guide/control");

  // Special function: Execute GUIDE_PERSON through its dedicated ROS action.
  SkillResultData guide(const Blackboard::SharedPtr &blackboard,
                        const ActionStepData &action);

 private:
  rclcpp::Node::SharedPtr node_;
  bool simulate_;
  std::string action_name_;
  rclcpp_action::Client<robot_ai_interfaces::action::GuidePerson>::SharedPtr
      client_;
};

class LifecycleResourceClient {
 public:
  // Special function: Create clients for the central lifecycle manager only.
  explicit LifecycleResourceClient(const rclcpp::Node::SharedPtr &node,
                                   double transition_timeout_sec = 60.0,
                                   double operation_margin_sec = 10.0);

  // Special function: Prepare every lifecycle node registered for a skill.
  std::pair<bool, std::string> start(const std::string &skill_name);

  // Special function: Cleanup every lifecycle node registered for a skill.
  std::pair<bool, std::string> stop(const std::string &skill_name);

  // Special function: Request registry-controlled recovery.
  std::pair<bool, std::string> recover(const std::string &skill_name);

 private:
  // Special function: Invoke one bounded lifecycle-manager service.
  std::pair<bool, std::string> call(
      const rclcpp::Client<robot_ai_interfaces::srv::ManageLifecycleSkill>::SharedPtr
          &client,
      const std::string &skill_name);

  rclcpp::Node::SharedPtr node_;
  double transition_timeout_sec_;
  double operation_margin_sec_;
  rclcpp::Client<robot_ai_interfaces::srv::ManageLifecycleSkill>::SharedPtr
      prepare_client_;
  rclcpp::Client<robot_ai_interfaces::srv::ManageLifecycleSkill>::SharedPtr
      cleanup_client_;
  rclcpp::Client<robot_ai_interfaces::srv::ManageLifecycleSkill>::SharedPtr
      recover_client_;
};

}  // namespace robot_yasmin_tasks

#endif  // ROBOT_YASMIN_TASKS__CLIENTS_HPP_
