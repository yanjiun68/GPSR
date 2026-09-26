// Section: Unit tests for GPSR skill-state factory routing and input validation.
#include <gtest/gtest.h>

#include "robot_yasmin_tasks/skills.hpp"

namespace robot_yasmin_tasks {
namespace {

// Special function: Construct only the local clients required by factory contract
// tests.
// Section: Test fixtures and state-machine execution helpers.
SkillClients local_clients() {
  SkillClients clients;
  clients.perception = std::make_shared<PerceptionClient>(nullptr, true);
  clients.manipulation = std::make_shared<ManipulationClient>(nullptr, true);
  clients.guide = std::make_shared<GuidePersonClient>(nullptr, true);
  clients.fixed_information =
      std::make_shared<FixedInformationClient>(nullptr, nullptr, true);
  return clients;
}

// Special function: Execute one factory-produced state machine with a fresh blackboard.
std::pair<std::string, Blackboard::SharedPtr> execute_action(
    const ActionStepData &action, const SkillClients &clients,
    const std::string &held_object = "") {
  auto blackboard = std::make_shared<Blackboard>();
  blackboard->set<ActionStepData>(bb::CURRENT_ACTION, action);
  if (!held_object.empty()) {
    blackboard->set<std::string>(bb::HELD_OBJECT, held_object);
  }
  auto state_machine = make_skill_state_machine(action.action_type, clients);
  return {(*state_machine)(blackboard), blackboard};
}

// Section: Manipulation validation must reject unsafe or incomplete geometry.
TEST(SkillFactory, PickRequiresPose) {
  ActionStepData action;
  action.action_type = PICK_OBJECT;
  action.target_object = "cup";
  const auto execution = execute_action(action, local_clients());
  EXPECT_EQ(execution.first, INVALID_INPUT);
  EXPECT_EQ(
      bb_get<SkillResultData>(execution.second, bb::LAST_SKILL_RESULT, {}).message,
      "Pick requires object pose.");
}

// Section: Placement must retain both held-object and destination preconditions.
TEST(SkillFactory, PlaceRequiresDestination) {
  ActionStepData action;
  action.action_type = PLACE_OBJECT;
  const auto execution = execute_action(action, local_clients(), "cup");
  EXPECT_EQ(execution.first, INVALID_INPUT);
  EXPECT_EQ(
      bb_get<SkillResultData>(execution.second, bb::LAST_SKILL_RESULT, {}).message,
      "Place requires target pose or location.");
}

// Section: The legacy destination parameter remains a valid guide target.
TEST(SkillFactory, GuideAcceptsDestinationParameter) {
  ActionStepData action;
  action.action_type = GUIDE_PERSON;
  action.target_person = "Alex";
  action.parameters = Json{{"destination", "kitchen"}};
  const auto execution = execute_action(action, local_clients());
  EXPECT_EQ(execution.first, SUCCESS);
}

// Section: Retained simple Python skills must be reachable from the C++ factory.
TEST(SkillFactory, CompareAndDisposeAreReachable) {
  ActionStepData compare;
  compare.action_type = COMPARE_OBJECTS;
  compare.parameters = Json{{"property", "largest"}, {"selected_object", "bottle"}};
  EXPECT_EQ(execute_action(compare, local_clients()).first, SUCCESS);

  ActionStepData dispose;
  dispose.action_type = DISPOSE_TRASH;
  EXPECT_EQ(execute_action(dispose, local_clients(), "trash").first, SUCCESS);
}

// Section: Fixed-information SAY requests must route to the simulated dialogue
// provider.
TEST(SkillFactory, FixedInformationSayIsReachable) {
  ActionStepData action;
  action.action_type = SAY;
  action.parameters = Json{{"question", "tell me today's date"},
                           {"fixed_information", true},
                           {"simulated_answer", "Today is Thursday."}};
  const auto execution = execute_action(action, local_clients());
  EXPECT_EQ(execution.first, SUCCESS);
  EXPECT_EQ(bb_get<std::string>(execution.second, bb::REPORT_TEXT, ""),
            "Today is Thursday.");
}

// Section: Invalid selectors and extraction fields must fail before remote action
// dispatch.
TEST(SkillFactory, RemoteInputsAreValidatedLocally) {
  ActionStepData attribute;
  attribute.action_type = FIND_PERSON_BY_ATTRIBUTE;
  attribute.parameters = Json{{"selector", {{"type", "name"}, {"value", "Alex"}}}};
  EXPECT_EQ(execute_action(attribute, local_clients()).first, INVALID_INPUT);

  ActionStepData extract;
  extract.action_type = EXTRACT_PERSON_INFO;
  extract.parameters = Json{{"field", "unsupported_field"}};
  EXPECT_EQ(execute_action(extract, local_clients()).first, INVALID_INPUT);
}

}  // namespace
}  // namespace robot_yasmin_tasks
