cmake_minimum_required(VERSION 3.8)
project(robot_ai_interfaces)

# Section: Load interface generation packages.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Section: Generate GPSR action queue interfaces.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionStep.msg"
  "msg/ActionQueue.msg"
  "msg/SkillResult.msg"
  "action/ExecuteActionQueue.action"
  "action/ExecuteTaskPlan.action"
  "action/FindObject.action"
  "action/FindPerson.action"
  "action/PickObject.action"
  "action/PlaceObject.action"
  "action/Handover.action"
  "action/FollowPerson.action"
  "action/GuidePerson.action"
  "action/AskAndListen.action"
  "action/CountObjects.action"
  "action/CountPeople.action"
  "srv/CountObjects.srv"
  "srv/CountPeople.srv"
  "srv/CompareObjects.srv"
  "srv/ExtractPersonInfo.srv"
  "srv/ManageLifecycleSkill.srv"
  "srv/GetLifecycleSkillState.srv"
  "srv/Say.srv"
  "srv/ScheduleCommand.srv"
  "srv/ValidateActionQueue.srv"
  DEPENDENCIES geometry_msgs
)

# Section: Export generated runtime dependencies.
ament_export_dependencies(rosidl_default_runtime)
ament_package()
