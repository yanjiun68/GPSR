cmake_minimum_required(VERSION 3.8)
project(person_follower_master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Track.msg"
  "msg/Tracks.msg"
  "msg/MasterTarget.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Utility module lives under the separate `pfm_utils` Python package so it
# does not collide with the `person_follower_master` package that rosidl's
# Python generator installs for the msg bindings.
ament_python_install_package(pfm_utils)

install(PROGRAMS
  scripts/reid_enricher_node.py
  scripts/master_tracker_node.py
  scripts/follow_controller_node.py
  scripts/cmd_vel_mux_node.py
  scripts/return_safety_wrapper_node.py
  scripts/session_manager_node.py
  scripts/person_follow_action_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

# Section: Pure policy tests guard completion arming and stop-gesture geometry.
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_follow_action_policy
    test/test_follow_action_policy.py
  )
endif()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
