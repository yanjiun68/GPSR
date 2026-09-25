cmake_minimum_required(VERSION 3.8)
project(robot_ai_interfaces)

# Section: Load interface generation packages.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

# Section: Generate consolidated GPSR, receptionist, and text-to-speech interfaces.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionStep.msg"
  "msg/ActionQueue.msg"
  "msg/SkillResult.msg"
  "msg/GpsrGuestRecord.msg"
  "msg/FoundEntityRecord.msg"
  "msg/CurrentGuestStatus.msg"
  "msg/GuestAppearance.msg"
  "msg/GuestCount.msg"
  "msg/PersonProfile.msg"
  "msg/RecognitionStatus.msg"
  "msg/TrackedPersonAppearance.msg"
  "msg/TrackedPersonAppearanceArray.msg"
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
  "action/FindPersonByAttribute.action"
  "action/ExtractGuestInfo.action"
  "action/ResolveVisualTarget.action"
  "action/CaptureNamedProfile.action"
  "action/CollectGuestProfile.action"
  "action/NavigateNamedPose.action"
  "action/RecognizeHumanId.action"
  "action/RunRcTask.action"
  "action/SpeakText.action"
  "action/TrackGuestAppearance.action"
  "srv/CountObjects.srv"
  "srv/CountPeople.srv"
  "srv/CompareObjects.srv"
  "srv/ExtractPersonInfo.srv"
  "srv/SaveGpsrGuest.srv"
  "srv/GetGpsrGuest.srv"
  "srv/ListGpsrGuests.srv"
  "srv/SaveFoundEntity.srv"
  "srv/LookupFoundEntity.srv"
  "srv/ManageLifecycleSkill.srv"
  "srv/GetLifecycleSkillState.srv"
  "srv/Say.srv"
  "srv/ScheduleCommand.srv"
  "srv/ValidateActionQueue.srv"
  "srv/CaptureAppearance.srv"
  "srv/ClearCurrentGuest.srv"
  "srv/CommitCurrentGuest.srv"
  "srv/FacePose.srv"
  "srv/FindEmptyChair.srv"
  "srv/GetGuestCount.srv"
  "srv/GetProfile.srv"
  "srv/LookupIdentityProfile.srv"
  "srv/MarkProfileSeated.srv"
  "srv/PointToSeat.srv"
  "srv/PrepareNavigationSession.srv"
  "srv/ResolvePresentationTargets.srv"
  "srv/SetCurrentGuestProfile.srv"
  "srv/SetHostProfile.srv"
  "srv/SpeakGuestIntroduction.srv"
  "srv/SpeakText.srv"
  DEPENDENCIES geometry_msgs builtin_interfaces sensor_msgs
)

# Section: Export generated runtime dependencies.
ament_export_dependencies(rosidl_default_runtime)
ament_package()
