cmake_minimum_required(VERSION 3.8)
project(rx200_gripper_guard_interfaces)

# Section: Generate only the action contract used between the guard and grasp node.
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CloseUntilContact.action"
  DEPENDENCIES action_msgs builtin_interfaces
)

# Section: Export generated runtime support to Python and C++ consumers.
ament_export_dependencies(rosidl_default_runtime)
ament_package()
