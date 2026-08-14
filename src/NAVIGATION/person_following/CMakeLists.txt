cmake_minimum_required(VERSION 3.8)
project(person_following)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/vision_node.py
  scripts/reid_test_node.py
  scripts/retinaface_person_crop_test.py
  scripts/follow_person.py
  scripts/person_track_manager.py
  scripts/target_lock_manager.py
  scripts/follow_behavior_manager.py
  scripts/safety_supervisor.py
  scripts/nav_follow_interface.py
  scripts/local_follow_interface.py
  scripts/boundary_zone_checker.py
  scripts/person_follow_nav2_goal.py
  scripts/person_follow_server.py
  scripts/person_follow_client.py
  scripts/follow_ready_voice_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
