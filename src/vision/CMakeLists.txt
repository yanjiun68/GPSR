cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(putrabot_interfaces REQUIRED)
find_package(receptionist_interfaces REQUIRED)
find_package(robot_ai_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(voice_interfaces REQUIRED)

# Section: Build the C++ equivalent of the external OpenCV camera publisher.
add_executable(external_camera_node_cpp src/external_camera_node.cpp)
ament_target_dependencies(external_camera_node_cpp rclcpp sensor_msgs std_msgs cv_bridge OpenCV)
target_link_libraries(external_camera_node_cpp ${OpenCV_LIBS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CamDetection.msg"
  "msg/CamDetections.msg"
  "msg/RecognizedPerson.msg"
  "srv/UpdatePersonProfile.srv"
  "action/RegisterPerson.action"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Install the renamed handwritten Python package independently of generated
# vision interfaces. Distinct names avoid duplicate ament Python targets.
ament_python_install_package(human_identity)

# Section: Install pure target mesh and grasp-geometry helpers for the perception-only node.
ament_python_install_package(target_geometry)

ament_export_dependencies(rosidl_default_runtime)


install(
  PROGRAMS
  scripts/vision_node.py
  scripts/external_camera_node.py
  scripts/find_object.py
  scripts/find_object_perception_server.py
  scripts/perception_memory_node.py
  scripts/gpsr_object_detection_node.py
  scripts/gesture_classifier.py
  scripts/human_skeleton_detection_node.py
  scripts/gpsr_human_identity_lifecycle_node.py
  scripts/count_objects_perception_server.py
  scripts/count_people_perception_server.py
  scripts/qwen_roi_depth_resolver.py
  scripts/target_grasp_geometry_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Section: Install the C++ external camera publisher beside the existing Python implementation.
install(
  TARGETS external_camera_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install the identity wrapper with the executable name used by its launch file.
install(
  PROGRAMS scripts/human_identity_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME human_identity_node
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY tools
  DESTINATION share/${PROJECT_NAME}
)

# Section: Install the dedicated polygon-target RViz configuration.
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    test_human_identity
    test/test_human_identity.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )

  # Section: Run pure-Python upper-body gesture classifier regression tests.
  ament_add_pytest_test(
    test_gesture_classifier
    test/test_gesture_classifier.py
  )

  # Section: Run ROS message and pose-node helper regression tests.
  ament_add_pytest_test(
    test_human_skeleton_detection_node
    test/test_human_skeleton_detection_node.py
  )

  # Section: Verify the demand-launched COUNT_OBJECTS stack owns only its three nodes.
  ament_add_pytest_test(
    test_gpsr_count_objects_skill_launch
    test/test_gpsr_count_objects_skill_launch.py
  )

  # Section: Verify Qwen ROI scaling, robust depth sampling, projection, and stand-off geometry.
  ament_add_pytest_test(
    test_qwen_roi_depth_resolver
    test/test_qwen_roi_depth_resolver.py
  )

  # Section: Verify grounding, masked projection, mesh validation, OBB, and grasp visualization.
  ament_add_pytest_test(
    test_target_grasp_geometry
    test/test_target_grasp_geometry.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
