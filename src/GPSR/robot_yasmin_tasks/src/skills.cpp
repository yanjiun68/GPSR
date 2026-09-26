// Section: YASMIN skill validation, execution states, and canonical skill-state
// factories.
#include "robot_yasmin_tasks/skills.hpp"

#include <stdexcept>

namespace robot_yasmin_tasks {
namespace {

// Special function: Require one action field or JSON parameter by canonical name.
// Section: Input validators that reject incomplete or unsupported skill requests before
// ROS dispatch.
ValidationCall require_any_field(std::vector<std::string> names) {
  return [names = std::move(names)](const Blackboard::SharedPtr &,
                                    const ActionStepData &action) {
    for (const auto &name : names) {
      std::string value;
      if (name == "target_location") value = action.target_location;
      if (name == "target_person") value = action.target_person;
      if (name == "target_object") value = action.target_object;
      if (name == "target_category") value = action.target_category;
      if (!value.empty() ||
          (action.parameters.is_object() && action.parameters.contains(name) &&
           !action.parameters.at(name).is_null() && action.parameters.at(name) != "")) {
        return std::make_pair(true, std::string{});
      }
    }
    std::string joined;
    for (const auto &name : names) {
      if (!joined.empty()) joined += ", ";
      joined += name;
    }
    return std::make_pair(false, "Missing one of required fields: " + joined);
  };
}

// Special function: Validate the complete attribute-selector contract before ROS
// dispatch.
std::pair<bool, std::string> validate_attribute_selector(const Blackboard::SharedPtr &,
                                                         const ActionStepData &action) {
  // Section: Named-person searches must use FIND_PERSON so identity semantics stay
  // explicit.
  if (!action.target_person.empty()) {
    return {false, "FIND_PERSON_BY_ATTRIBUTE cannot accept a name selector."};
  }
  if (!action.parameters.contains("selector") ||
      !action.parameters.at("selector").is_object() ||
      action.parameters.at("selector").empty()) {
    return {false, "At least one attribute selector is required."};
  }
  const auto &selector = action.parameters.at("selector");
  Json predicates =
      selector.contains("all") ? selector.at("all") : Json::array({selector});
  if (!predicates.is_array() || predicates.empty()) {
    return {false, "At least one attribute selector is required."};
  }
  static const std::set<std::string> supported{
      "pose", "gesture", "any_person", "shirt_color", "glasses", "hat", "face_mask"};
  static const std::set<std::string> affirmative{"yes", "true", "wearing", "1"};
  for (const auto &predicate : predicates) {
    if (!predicate.is_object()) {
      return {false, "Every attribute selector must be an object."};
    }
    auto type = predicate.value("type", std::string{});
    auto value = predicate.value("value", std::string{});
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    if (type == "name") {
      return {false, "FIND_PERSON_BY_ATTRIBUTE cannot accept a name selector."};
    }
    if (supported.count(type) == 0U) {
      return {false, "Unsupported person attribute selector: " +
                         (type.empty() ? std::string("empty") : type) + "."};
    }
    if (type != "any_person" && value.empty()) {
      return {false, "Attribute selector requires a non-empty value."};
    }
    if ((type == "glasses" || type == "hat" || type == "face_mask") &&
        affirmative.count(value) == 0U) {
      return {false, "Negative " + type + " matching is not supported."};
    }
  }
  return {true, ""};
}

// Special function: Validate every supported EXTRACT_PERSON_INFO field locally.
std::pair<bool, std::string> validate_person_info_fields(const Blackboard::SharedPtr &,
                                                         const ActionStepData &action) {
  // Section: An omitted field requests the complete supported information set.
  Json fields = action.parameters.value("fields", Json::array());
  if ((!fields.is_array() || fields.empty()) && action.parameters.contains("field")) {
    fields = Json::array({action.parameters.at("field")});
  }
  if (!fields.is_array()) {
    return {false, "Invalid person info field."};
  }
  static const std::set<std::string> supported{
      "name",      "age",         "gender",       "favorite_drink", "favourite_drink",
      "hometown",  "shirt_color", "shirt_colour", "glasses",        "hat",
      "face_mask", "pose",        "gesture",      "profile_id",     "person_id",
      "track_id"};
  for (const auto &field : fields) {
    if (!field.is_string()) {
      return {false, "Invalid person info field."};
    }
    auto normalized = field.get<std::string>();
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), ::tolower);
    if (supported.count(normalized) == 0U) {
      return {false, "Invalid person info field."};
    }
  }
  return {true, ""};
}

// Special function: Read the current normalized ActionStep from the blackboard.
ActionStepData current_action(const Blackboard::SharedPtr &blackboard) {
  return bb_get<ActionStepData>(blackboard, bb::CURRENT_ACTION, ActionStepData{});
}

// Special function: Normalize one desired scan heading into a relative signed rotation.
double scan_delta_radians(double desired_deg, double current_deg) {
  // Section: Keep every turn within the shortest signed full-circle interval.
  double delta = std::fmod(desired_deg - current_deg + 180.0, 360.0);
  if (delta < 0.0) delta += 360.0;
  delta -= 180.0;
  return delta * 3.14159265358979323846 / 180.0;
}

// Special function: Execute a PersonWaypointSearchSM-style route around one local
// perception callback.
SkillResultData search_with_waypoint_route(
    const Blackboard::SharedPtr &blackboard, const ActionStepData &original,
    const std::vector<std::string> &global_waypoints,
    const std::vector<double> &scan_headings_deg, double observation_timeout_sec,
    double navigation_timeout_sec, double scan_timeout_sec,
    double default_mission_timeout_sec, const std::string &entity_name,
    const std::string &preferred_location, const SkillClients &clients,
    const SkillClientCall &local_search) {
  // Section: Resolve strict-location versus global route semantics before any
  // movement or perception request.
  std::string requested = original.target_location;
  if (requested.empty()) {
    requested = original.parameters.value("target_location", std::string{});
  }
  if (requested.empty()) {
    requested = original.parameters.value("location", std::string{});
  }
  auto scope = original.parameters.value(
      "search_scope",
      requested.empty() ? std::string("GLOBAL") : std::string("STRICT_LOCATION"));
  std::transform(scope.begin(), scope.end(), scope.begin(), ::toupper);
  std::vector<std::string> waypoints;
  if (scope == "STRICT_LOCATION" && !requested.empty()) {
    waypoints = {requested};
  } else if (scope == "GLOBAL" && requested.empty()) {
    waypoints = global_waypoints;
    const auto iterator =
        std::find(waypoints.begin(), waypoints.end(), preferred_location);
    if (iterator != waypoints.end()) {
      waypoints.erase(iterator);
      waypoints.insert(waypoints.begin(), preferred_location);
    }
  } else {
    return make_skill_result(
        original.action_type, INVALID_INPUT,
        scope == "GLOBAL"
            ? "GLOBAL " + entity_name + " search cannot include target_location."
            : "STRICT_LOCATION " + entity_name +
                  " search requires target_location.",
        Json::object(), false, true);
  }
  if (waypoints.empty()) {
    return make_skill_result(original.action_type, INVALID_INPUT,
                             entity_name + " search route contains no waypoints.",
                             Json::object(), false, true);
  }

  // Section: One absolute deadline bounds navigation, observation, and scanning
  // together.
  const double mission_timeout = original.timeout_sec > 0.0
                                     ? original.timeout_sec
                                     : default_mission_timeout_sec;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(std::max(0.1, mission_timeout));
  Json visited = Json::array();
  Json navigation_failures = Json::array();
  Json scan_failures = Json::array();
  for (std::size_t index = 0; index < waypoints.size(); ++index) {
    if (cancel_requested(blackboard)) {
      return make_skill_result(original.action_type, CANCELED,
                               entity_name + " search canceled.", Json::object(),
                               false, true);
    }
    auto remaining =
        std::chrono::duration<double>(deadline - std::chrono::steady_clock::now())
            .count();
    if (remaining <= 0.0) {
      return make_skill_result(original.action_type, TIMEOUT,
                               entity_name +
                                   " search exceeded its mission deadline.",
                               Json{{"searched_waypoints", visited}}, true);
    }
    ActionStepData navigation = original;
    navigation.action_type = NAVIGATE;
    navigation.target_location = waypoints[index];
    navigation.parameters = Json::object();
    navigation.timeout_sec = std::min(navigation_timeout_sec, remaining);
    const auto navigation_result =
        clients.navigation->navigate_to_pose(blackboard, navigation);
    if (navigation_result.outcome == CANCELED || navigation_result.outcome == TIMEOUT) {
      auto result = navigation_result;
      result.action_type = original.action_type;
      return result;
    }
    if (navigation_result.outcome != SUCCESS) {
      navigation_failures.push_back(
          Json{{"waypoint", waypoints[index]}, {"message", navigation_result.message}});
      continue;
    }
    visited.push_back(waypoints[index]);

    // Section: Observe on arrival and after each configured absolute scan heading.
    double current_heading = 0.0;
    std::vector<double> headings{0.0};
    headings.insert(headings.end(), scan_headings_deg.begin(),
                    scan_headings_deg.end());
    for (const double desired_heading : headings) {
      remaining =
          std::chrono::duration<double>(deadline - std::chrono::steady_clock::now())
              .count();
      if (remaining <= 0.0) {
        return make_skill_result(original.action_type, TIMEOUT,
                                 entity_name +
                                     " search exceeded its mission deadline.",
                                 Json{{"searched_waypoints", visited}}, true);
      }
      if (desired_heading != 0.0) {
        const auto spin = clients.navigation->spin(
            blackboard, original, scan_delta_radians(desired_heading, current_heading),
            std::min(scan_timeout_sec, remaining));
        if (spin.outcome == CANCELED || spin.outcome == TIMEOUT) {
          auto result = spin;
          result.action_type = original.action_type;
          return result;
        }
        if (spin.outcome != SUCCESS) {
          scan_failures.push_back(Json{{"waypoint", waypoints[index]},
                                       {"heading_deg", desired_heading},
                                       {"message", spin.message}});
          break;
        }
        current_heading = desired_heading;
      }
      ActionStepData local = original;
      local.target_location = waypoints[index];
      local.parameters["active_search_waypoint"] = waypoints[index];
      // Section: Each server request is a bounded local observation because this
      // state machine owns the global route.
      local.parameters["search_scope"] = "STRICT_LOCATION";
      local.timeout_sec = std::min(
          observation_timeout_sec,
          std::chrono::duration<double>(deadline - std::chrono::steady_clock::now())
              .count());
      auto perception = local_search(blackboard, local);
      if (perception.outcome == SUCCESS) {
        perception.action_type = original.action_type;
        perception.payload["searched_waypoints"] = visited;
        perception.payload["navigation_failures"] = navigation_failures;
        perception.payload["scan_failures"] = scan_failures;
        return perception;
      }
      if (perception.outcome != NOT_FOUND) {
        perception.action_type = original.action_type;
        return perception;
      }
    }
  }
  // Section: Distinguish an exhausted route from a route with no reachable waypoint.
  if (visited.empty()) {
    return make_skill_result(original.action_type, FAILED,
                             "No configured " + entity_name +
                                 "-search waypoint was reachable.",
                             Json{{"searched_waypoints", visited},
                                  {"navigation_failures", navigation_failures}},
                             true);
  }
  return make_skill_result(
      original.action_type, NOT_FOUND,
      entity_name +
          " was not found after all configured waypoints were searched.",
      Json{{"searched_waypoints", visited},
           {"navigation_failures", navigation_failures},
           {"scan_failures", scan_failures}},
      true);
}

// Special function: Execute the former PersonWaypointSearchSM mission around
// attribute perception.
SkillResultData find_person_by_attribute_with_route(
    const Blackboard::SharedPtr &blackboard, const ActionStepData &original,
    const SkillClients &clients) {
  // Section: Attribute search prioritizes a remembered person location when present.
  const auto preferred =
      original.parameters.value("preferred_location", std::string{});
  return search_with_waypoint_route(
      blackboard, original, clients.person_search.global_waypoints,
      clients.person_search.scan_headings_deg,
      clients.person_search.observation_timeout_sec,
      clients.person_search.navigation_timeout_sec,
      clients.person_search.scan_timeout_sec,
      clients.person_search.mission_timeout_sec, "Person", preferred, clients,
      [client = clients.remote](const auto &board, const auto &action) {
        return client->find_person_by_attribute(board, action);
      });
}

// Special function: Execute a PersonWaypointSearchSM-style mission around local
// object perception.
SkillResultData find_object_with_route(const Blackboard::SharedPtr &blackboard,
                                       const ActionStepData &original,
                                       const SkillClients &clients) {
  // Section: Object history may prioritize a previously successful viewpoint without
  // reducing route coverage.
  auto preferred =
      original.parameters.value("preferred_viewpoint", std::string{});
  if (preferred.empty()) {
    preferred = original.parameters.value("preferred_location", std::string{});
  }
  return search_with_waypoint_route(
      blackboard, original, clients.object_search.global_waypoints,
      clients.object_search.scan_headings_deg,
      clients.object_search.observation_timeout_sec,
      clients.object_search.navigation_timeout_sec,
      clients.object_search.scan_timeout_sec,
      clients.object_search.mission_timeout_sec, "Object", preferred, clients,
      [client = clients.remote](const auto &board, const auto &action) {
        return client->find_object(board, action);
      });
}

}  // namespace

// Section: Reusable YASMIN states implementing validation and client execution
// transitions.
ValidateSkillInputState::ValidateSkillInputState(ValidationCall validator)
    : yasmin::State({SUCCESS, INVALID_INPUT}), validator_(std::move(validator)) {}

std::string ValidateSkillInputState::execute(Blackboard::SharedPtr blackboard) {
  // Section: Preserve negative-timeout rejection before skill-specific validation.
  const auto action = current_action(blackboard);
  blackboard->set<ActionStepData>(bb::CURRENT_ACTION, action);
  std::pair<bool, std::string> validation{true, ""};
  if (action.timeout_sec < 0.0) {
    validation = {false, "Timeout must not be negative."};
  } else if (validator_) {
    validation = validator_(blackboard, action);
  }
  if (validation.first) {
    return SUCCESS;
  }
  const auto result = make_skill_result(action.action_type, INVALID_INPUT,
                                        validation.second, Json::object(), false, true);
  blackboard->set<std::string>(bb::FAILURE_REASON, validation.second);
  blackboard->set<SkillResultData>(bb::LAST_SKILL_RESULT, result);
  return INVALID_INPUT;
}

ExecuteSkillClientState::ExecuteSkillClientState(SkillClientCall client_call)
    : yasmin::State(skill_outcomes()), client_call_(std::move(client_call)) {}

std::string ExecuteSkillClientState::execute(Blackboard::SharedPtr blackboard) {
  // Section: Convert exceptions into the same recoverable failed result.
  const auto action = current_action(blackboard);
  SkillResultData result;
  try {
    result = client_call_(blackboard, action);
  } catch (const std::exception &error) {
    result = make_skill_result(action.action_type, FAILED,
                               std::string("Skill client error: ") + error.what(),
                               Json::object(), true);
  }
  blackboard->set<SkillResultData>(bb::LAST_SKILL_RESULT, result);
  blackboard->set<std::string>(bb::DISPATCH_OUTCOME, result.outcome);
  return result.outcome;
}

SimpleSkillSM::SimpleSkillSM(const std::string &name, SkillClientCall client_call,
                             ValidationCall validator)
    : yasmin::StateMachine(name, skill_outcomes()) {
  // Section: Keep original state names and transition topology.
  add_state("VALIDATE_INPUT",
            std::make_shared<ValidateSkillInputState>(std::move(validator)),
            {{SUCCESS, "EXECUTE_CLIENT"}, {INVALID_INPUT, INVALID_INPUT}});
  yasmin::Transitions transitions;
  for (const auto &outcome : skill_outcomes()) {
    transitions[outcome] = outcome;
  }
  add_state("EXECUTE_CLIENT",
            std::make_shared<ExecuteSkillClientState>(std::move(client_call)),
            transitions);
}

// Section: Canonical action-to-state-machine routing for every GPSR primitive.
yasmin::StateMachine::SharedPtr make_skill_state_machine(const std::string &action_type,
                                                         const SkillClients &clients) {
  // Section: Preserve one explicit mapping per canonical primitive.
  if (action_type == NAVIGATE) {
    return std::make_shared<SimpleSkillSM>(
        "NAVIGATION_SKILL_SM",
        [client = clients.navigation](const auto &bb, const auto &action) {
          return client->navigate_to_pose(bb, action);
        },
        require_any_field({"target_location", "target_pose"}));
  }
  if (action_type == COUNT_OBJECTS) {
    return std::make_shared<SimpleSkillSM>(
        "COUNT_OBJECTS_SKILL_SM",
        [client = clients.perception](const auto &bb, const auto &action) {
          return client->count_objects(bb, action);
        });
  }
  if (action_type == COUNT_PEOPLE) {
    return std::make_shared<SimpleSkillSM>(
        "COUNT_PEOPLE_SKILL_SM",
        [client = clients.perception](const auto &bb, const auto &action) {
          return client->count_people(bb, action);
        });
  }
  if (action_type == FIND_OBJECT) {
    return std::make_shared<SimpleSkillSM>(
        "FIND_OBJECT_SKILL_SM",
        [clients](const auto &bb, const auto &action) {
          return find_object_with_route(bb, action, clients);
        },
        require_any_field(
            {"target_object", "object_name", "target_category", "category"}));
  }
  if (action_type == FIND_PERSON_BY_ATTRIBUTE) {
    return std::make_shared<SimpleSkillSM>(
        "FIND_PERSON_BY_ATTRIBUTE_SM",
        [clients](const auto &bb, const auto &action) {
          return find_person_by_attribute_with_route(bb, action, clients);
        },
        validate_attribute_selector);
  }
  if (action_type == EXTRACT_PERSON_INFO) {
    return std::make_shared<SimpleSkillSM>(
        "EXTRACT_PERSON_INFO_SKILL_SM",
        [client = clients.remote](const auto &bb, const auto &action) {
          return client->extract_guest_info(bb, action);
        },
        validate_person_info_fields);
  }
  if (action_type == SAY) {
    return std::make_shared<SimpleSkillSM>(
        "SAY_SKILL_SM",
        [speech = clients.speech, fixed_information = clients.fixed_information](
            const auto &bb, const auto &action) {
          // Section: Fixed facts stay a SAY primitive, but their answer is owned by the
          // dialogue provider.
          if (action.parameters.value("fixed_information", false)) {
            if (!fixed_information) {
              return make_skill_result(action.action_type, FAILED,
                                       "Fixed-information client unavailable.",
                                       Json::object(), true);
            }
            return fixed_information->answer(bb, action);
          }
          // Section: Ordinary SAY actions retain direct text-to-speech behavior.
          if (!speech) {
            return make_skill_result(action.action_type, FAILED,
                                     "Speech client unavailable.", Json::object(),
                                     true);
          }
          return speech->say(bb, action);
        },
        require_any_field({"text", "text_template", "question"}));
  }
  if (action_type == ASK_AND_LISTEN) {
    return std::make_shared<SimpleSkillSM>(
        "DIALOGUE_SKILL_SM",
        [client = clients.dialogue](const auto &bb, const auto &action) {
          return client->ask_and_listen(bb, action);
        },
        require_any_field({"question", "text"}));
  }
  if (action_type == FIXED_INFORMATION) {
    // Section: Accept legacy queues while routing execution through the unified SAY
    // state machine.
    return std::make_shared<SimpleSkillSM>(
        "SAY_SKILL_SM",
        [client = clients.fixed_information](const auto &bb, const auto &action) {
          return client->answer(bb, action);
        },
        require_any_field({"question", "text"}));
  }
  if (action_type == FOLLOW_PERSON) {
    return std::make_shared<SimpleSkillSM>(
        "FOLLOW_PERSON_SKILL_SM",
        [client = clients.follow](const auto &bb, const auto &action) {
          return client->follow(bb, action);
        },
        require_any_field({"target_person"}));
  }
  if (action_type == PICK_OBJECT) {
    const auto validator = [](const Blackboard::SharedPtr &bb,
                              const ActionStepData &action) {
      if (!bb_get<std::string>(bb, bb::HELD_OBJECT, "").empty()) {
        return std::make_pair(false, std::string("Robot already holds an object."));
      }
      const bool target = !action.target_object.empty() ||
                          !bb_get<std::string>(bb, bb::TARGET_OBJECT, "").empty();
      if (!target) {
        return std::make_pair(false, std::string("Pick requires object reference."));
      }
      const bool pose = bb->contains(bb::TARGET_POSE) ||
                        (action.parameters.contains("object_pose") &&
                         !action.parameters.at("object_pose").is_null());
      return std::make_pair(pose, std::string("Pick requires object pose."));
    };
    return std::make_shared<SimpleSkillSM>(
        "PICK_OBJECT_SKILL_SM",
        [client = clients.manipulation](const auto &bb, const auto &action) {
          return client->pick_object(bb, action);
        },
        validator);
  }
  if (action_type == PLACE_OBJECT) {
    const auto validator = [](const Blackboard::SharedPtr &bb,
                              const ActionStepData &action) {
      const bool held = !bb_get<std::string>(bb, bb::HELD_OBJECT, "").empty();
      if (!held) {
        return std::make_pair(false, std::string("Place requires held_object."));
      }
      const bool destination = bb->contains(bb::TARGET_POSE) ||
                               !action.target_location.empty() ||
                               (action.parameters.contains("target_pose") &&
                                !action.parameters.at("target_pose").is_null());
      return std::make_pair(destination,
                            std::string("Place requires target pose or location."));
    };
    return std::make_shared<SimpleSkillSM>(
        "PLACE_OBJECT_SKILL_SM",
        [client = clients.manipulation](const auto &bb, const auto &action) {
          return client->place_object(bb, action);
        },
        validator);
  }
  if (action_type == HANDOVER_OBJECT) {
    const auto validator = [](const Blackboard::SharedPtr &bb,
                              const ActionStepData &action) {
      if (bb_get<std::string>(bb, bb::HELD_OBJECT, "").empty()) {
        return std::make_pair(false, std::string("No held object to hand over."));
      }
      const bool person = !action.target_person.empty() ||
                          !bb_get<std::string>(bb, bb::TARGET_PERSON, "").empty();
      return std::make_pair(person, std::string("Handover requires recipient."));
    };
    return std::make_shared<SimpleSkillSM>(
        "HANDOVER_SKILL_SM",
        [client = clients.manipulation](const auto &bb, const auto &action) {
          return client->handover(bb, action);
        },
        validator);
  }
  if (action_type == GUIDE_PERSON) {
    const auto validator = [](const Blackboard::SharedPtr &,
                              const ActionStepData &action) {
      const bool destination =
          !action.target_location.empty() ||
          (action.parameters.contains("destination") &&
           action.parameters.at("destination").is_string() &&
           !action.parameters.at("destination").get<std::string>().empty());
      return std::make_pair(!action.target_person.empty() && destination,
                            std::string("Guide requires person and destination."));
    };
    return std::make_shared<SimpleSkillSM>(
        "GUIDE_PERSON_SKILL_SM",
        [client = clients.guide](const auto &bb, const auto &action) {
          return client->guide(bb, action);
        },
        validator);
  }
  if (action_type == COMPARE_OBJECTS) {
    const auto validator = [](const Blackboard::SharedPtr &,
                              const ActionStepData &action) {
      auto property = action.parameters.value("property", std::string{});
      std::transform(property.begin(), property.end(), property.begin(), ::tolower);
      static const std::set<std::string> supported{"biggest",  "largest",  "smallest",
                                                   "heaviest", "lightest", "thinnest"};
      return std::make_pair(supported.count(property) != 0U,
                            std::string("Invalid comparison property."));
    };
    return std::make_shared<SimpleSkillSM>(
        "COMPARE_OBJECTS_SKILL_SM",
        [client = clients.perception](const auto &bb, const auto &action) {
          return client->compare_objects(bb, action);
        },
        validator);
  }
  if (action_type == DISPOSE_TRASH) {
    const auto validator = [](const Blackboard::SharedPtr &bb,
                              const ActionStepData &action) {
      const bool target = !bb_get<std::string>(bb, bb::HELD_OBJECT, "").empty() ||
                          !action.target_object.empty();
      return std::make_pair(target,
                            std::string("Dispose requires held or target object."));
    };
    return std::make_shared<SimpleSkillSM>(
        "DISPOSE_TRASH_SKILL_SM",
        [client = clients.manipulation](const auto &bb, const auto &action) {
          return client->dispose_trash(bb, action);
        },
        validator);
  }
  throw std::invalid_argument("Unsupported action type: " + action_type);
}

}  // namespace robot_yasmin_tasks
