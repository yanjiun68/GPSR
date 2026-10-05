cmake_minimum_required(VERSION 3.8)
project(rx200_handbag_grasp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_IS_CLANG)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveBaseDistance.action"
)

# Install Python scripts
install(PROGRAMS
  scripts/yolo_handle_detector.py
  scripts/grasp_node.py
  scripts/camera_tf_publisher.py
  scripts/yolo_viewer.py
  scripts/hand_bag_selector.py
  scripts/mediapipe_bag_viewer.py
  scripts/all_in_one_orchestrator.py
  scripts/autonomous_reach_grab_coordinator.py
  scripts/startup_lift_to_position.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch, config, urdf, etc.
install(DIRECTORY
  launch
  config
  models
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(FALSE) # Section: Tests disabled for production-only workspace builds.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
