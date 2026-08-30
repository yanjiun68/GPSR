cmake_minimum_required(VERSION 3.8)
project(waypoint_saver)

# Section: Load the build system and ROS interface generators.
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Section: Generate the service used to save a named current pose.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveWaypoint.srv"
  DEPENDENCIES geometry_msgs
)

# Section: Install the isolated waypoint-saver executable and pure helper module.
install(PROGRAMS
  scripts/waypoint_saver_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/waypoint_saver_logic.py
  DESTINATION lib/${PROJECT_NAME}
)

# Section: Install the standalone launch contract and operator documentation.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

# Section: Register hardware-free Python tests when colcon testing is enabled.
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_waypoint_saver_logic
    test/test_waypoint_saver_logic.py
  )
  ament_add_pytest_test(
    test_waypoint_saver_mocked_tf
    test/test_waypoint_saver_mocked_tf.py
  )
  ament_add_pytest_test(
    test_navigation_launch_paths
    test/test_navigation_launch_paths.py
  )
endif()

# Section: Export generated service runtime support to ROS clients.
ament_export_dependencies(rosidl_default_runtime)
ament_package()
