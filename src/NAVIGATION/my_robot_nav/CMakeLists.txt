cmake_minimum_required(VERSION 3.8)
project(my_robot_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(
    DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(
    DIRECTORY meshes/
    DESTINATION share/${PROJECT_NAME}/meshes
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Section: Register focused Python regressions for bootstrap localization safety.
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_lidar_loc_safety test/test_lidar_loc_safety.py)
  # Section: Verify Nav2 STARTUP state classification and duplicate suppression.
  ament_add_pytest_test(
    test_nav2_autostart_gate
    test/test_nav2_autostart_gate.py
  )
endif()

install(DIRECTORY
  behavior_trees
  config
  launch
  maps
  urdf
  world
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/capture_rviz_semantic_location.py
  src/base_driver.py
  src/breadcrumb_teach_repeat.py
  src/navigate_named_pose_server.py
  src/cmd_vel_to_targets.py
  src/kinematics_keypad.py
  src/initial_pose_publisher.py
  src/initial_pose_relay.py
  src/bootstrap_pose_promoter.py
  src/imu_decimator.py
  src/kinematics_tester.py
  src/lidar_loc.py
  src/amcl_global_localizer.py
  src/nav2_autostart_gate.py
  src/omni_odometry.py
  src/scan_fixer.py
  src/return_home.py
  src/scan_self_filter.py
  src/robocup_astra_camera_node.py
  src/astra_cam_qw.py
  src/semantic_navigator.py
  scripts/follow_person.py
  scripts/people_candidates_bridge.py
  scripts/person_follow_nav2_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
