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

# Section: Install only the isolated waypoint-saver executable and documentation.
install(PROGRAMS
  scripts/waypoint_saver_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

# Section: Export generated service runtime support to ROS clients.
ament_export_dependencies(rosidl_default_runtime)
ament_package()
