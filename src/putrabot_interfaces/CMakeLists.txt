cmake_minimum_required(VERSION 3.8)
project(putrabot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BehaviorStatus.msg"
  "msg/CamDetection.msg"
  "msg/CamDetections.msg"
  "msg/HumanSkeleton.msg"
  "msg/HumanSkeletonKeypoint.msg"
  "msg/HumanSkeletonLink.msg"
  "msg/HumanSkeletons.msg"
  "msg/HumanPose.msg"
  "msg/HumanPoseArray.msg"
  "msg/PoseLandmark.msg"
  "msg/LidarCluster.msg"
  "msg/LidarClusters.msg"
  "msg/MotionIntent.msg"
  "msg/ObjectDetection.msg"
  "msg/ObjectDetections.msg"
  "msg/SafetyStatus.msg"
  "msg/TargetLockDebug.msg"
  "msg/VisibleItems.msg"
  "msg/TrackedPose.msg"
  "msg/TrackedPoseArray.msg"
  "srv/SelectPerson.srv"
  "srv/ClearPerson.srv"
  "srv/GetPersonFollowStatus.srv"
  "srv/KillManagedProcess.srv"
  "srv/SetFollowParams.srv"
  "srv/SaveWaypoints.srv"
  "srv/GetWaypoints.srv"
  "srv/GetVisibleItems.srv"
  "action/AnnounceFollowReady.action"
  "action/ManagedProcess.action"
  "action/PersonFollow.action"
  "action/SelectBag.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_package()
