cmake_minimum_required(VERSION 3.8)
project(receptionist_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GuestAppearance.msg"
  "msg/PersonProfile.msg"
  "msg/CurrentGuestStatus.msg"
  "msg/GuestCount.msg"
  "msg/RecognitionStatus.msg"
  "msg/TrackedPersonAppearance.msg"
  "msg/TrackedPersonAppearanceArray.msg"
  "srv/SetHostProfile.srv"
  "srv/SetCurrentGuestProfile.srv"
  "srv/CommitCurrentGuest.srv"
  "srv/ClearCurrentGuest.srv"
  "srv/GetProfile.srv"
  "srv/LookupIdentityProfile.srv"
  "srv/GetGuestCount.srv"
  "srv/CaptureAppearance.srv"
  "srv/SpeakGuestIntroduction.srv"
  "srv/MarkProfileSeated.srv"
  "srv/SpeakText.srv"
  "srv/ResolvePresentationTargets.srv"
  "srv/FindEmptyChair.srv"
  "srv/PointToSeat.srv"
  "srv/PrepareNavigationSession.srv"
  "srv/FacePose.srv"
  "action/CollectGuestProfile.action"
  "action/CaptureNamedProfile.action"
  "action/RecognizeHumanId.action"
  "action/TrackGuestAppearance.action"
  "action/NavigateNamedPose.action"
  "action/RunRcTask.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
