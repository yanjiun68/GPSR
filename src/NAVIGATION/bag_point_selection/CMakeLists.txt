cmake_minimum_required(VERSION 3.8)
project(bag_point_selection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(putrabot_interfaces REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  bag_point_selection/bag_image_preprocessor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bag_image_preprocessor_node
)
install(PROGRAMS
  bag_point_selection/pose_pointing_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_pointing_node
)
install(PROGRAMS
  bag_point_selection/bag_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bag_detector_node
)
install(PROGRAMS
  bag_point_selection/selected_bag_cloud_cluster_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME selected_bag_cloud_cluster_node
)
install(PROGRAMS
  bag_point_selection/lidar_cluster_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_cluster_node
)
install(PROGRAMS
  bag_point_selection/target_fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME target_fusion_node
)
install(PROGRAMS
  bag_point_selection/selection_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME selection_manager_node
)
install(PROGRAMS
  bag_point_selection/bag_approach_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bag_approach_controller_node
)
install(PROGRAMS
  bag_point_selection/final_alignment_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME final_alignment_node
)
install(PROGRAMS
  bag_point_selection/bag_selection_action_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bag_selection_action_server
)
install(PROGRAMS
  bag_point_selection/bag_selection_voice_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bag_selection_voice_node
)
install(PROGRAMS
  bag_point_selection/bag_selection_viz_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bag_selection_viz_node
)
install(PROGRAMS
  bag_point_selection/side_motion_executor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME side_motion_executor_node
)

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "build" EXCLUDE
  PATTERN "install" EXCLUDE
  PATTERN "log" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_target_fusion_node
    test/test_target_fusion_node.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
