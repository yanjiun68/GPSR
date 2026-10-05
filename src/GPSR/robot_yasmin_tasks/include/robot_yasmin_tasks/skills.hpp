#ifndef ROBOT_YASMIN_TASKS__SKILLS_HPP_
#define ROBOT_YASMIN_TASKS__SKILLS_HPP_

#include "robot_yasmin_tasks/clients.hpp"

#include <cstdint>
#include <yasmin/state.hpp>
#include <yasmin/state_machine.hpp>

namespace robot_yasmin_tasks {

using ValidationCall = std::function<std::pair<bool, std::string>(
    const Blackboard::SharedPtr &, const ActionStepData &)>;

class ValidateSkillInputState final : public yasmin::State {
 public:
  // Special function: Store the existing optional skill validator.
  explicit ValidateSkillInputState(ValidationCall validator = {});

  // Special function: Normalize and validate the current ActionStep.
  std::string execute(Blackboard::SharedPtr blackboard) override;

 private:
  ValidationCall validator_;
};

class ExecuteSkillClientState final : public yasmin::State {
 public:
  // Special function: Store one deterministic client callback.
  explicit ExecuteSkillClientState(SkillClientCall client_call);

  // Special function: Execute the callback and publish its blackboard result.
  std::string execute(Blackboard::SharedPtr blackboard) override;

 private:
  SkillClientCall client_call_;
};

class SimpleSkillSM : public yasmin::StateMachine {
 public:
  // Special function: Recreate the Python VALIDATE_INPUT to EXECUTE_CLIENT graph.
  SimpleSkillSM(const std::string &name, SkillClientCall client_call,
                ValidationCall validator = {});
};

// Section: Normalized person-search contracts keep scheduler semantics separate from
// ROS transport-specific selector JSON.
struct PersonAttributeData {
  std::string type;
  std::string value;
  double minimum_confidence{0.0};
};

struct PersonQueryData {
  std::optional<std::string> name;
  std::optional<std::string> location;
  std::vector<PersonAttributeData> attributes;
  std::string verification_mode;
};

struct PersonCandidateData {
  std::string tracking_session_id;
  std::int32_t track_id{-1};
  std::string track_key;
  geometry_msgs::msg::PoseStamped pose;
  double confidence{0.0};
  std::string verification_source;
};

// Section: Immutable navigation policy used by compound person-search skills.
struct PersonSearchPolicyData {
  // Section: One bounded semantic rescan is the default person-search recovery.
  std::uint32_t max_not_found_retries{1};
  std::vector<std::string> global_waypoints{
      "kitchen", "living_room", "dining_room", "bedroom", "entrance", "exit"};
  std::vector<double> scan_headings_deg{60.0, -60.0, 120.0, -120.0, 180.0};
  double observation_timeout_sec{20.0};
  double navigation_timeout_sec{60.0};
  double scan_timeout_sec{15.0};
  double mission_timeout_sec{300.0};
};

// Section: Immutable navigation policy used by the compound object-search skill.
struct ObjectSearchPolicyData {
  // Section: A semantic retry repeats one complete route only after NOT_FOUND.
  std::uint32_t max_not_found_retries{1};
  std::vector<std::string> global_waypoints{
      "kitchen", "dining_room", "living_room", "bedroom", "entrance"};
  std::vector<double> scan_headings_deg{60.0, -60.0, 120.0, -120.0, 180.0};
  double observation_timeout_sec{3.0};
  double navigation_timeout_sec{90.0};
  double scan_timeout_sec{20.0};
  double mission_timeout_sec{300.0};
};

// Section: Runtime dependencies shared by skill factory instances.
struct SkillClients {
  std::shared_ptr<SpeechClient> speech;
  std::shared_ptr<DialogueClient> dialogue;
  std::shared_ptr<FixedInformationClient> fixed_information;
  std::shared_ptr<Nav2Client> navigation;
  std::shared_ptr<PerceptionClient> perception;
  std::shared_ptr<FollowPersonClient> follow;
  std::shared_ptr<RemoteSkillClient> remote;
  std::shared_ptr<ManipulationClient> manipulation;
  std::shared_ptr<GuidePersonClient> guide;
  PersonSearchPolicyData person_search;
  ObjectSearchPolicyData object_search;
};

// Special function: Build the state machine corresponding to one canonical primitive.
yasmin::StateMachine::SharedPtr make_skill_state_machine(
    const std::string &action_type, const SkillClients &clients);

}  // namespace robot_yasmin_tasks

#endif  // ROBOT_YASMIN_TASKS__SKILLS_HPP_
