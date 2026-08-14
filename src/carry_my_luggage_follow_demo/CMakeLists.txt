cmake_minimum_required(VERSION 3.8)
project(carry_my_luggage_follow_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(smacc2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(putrabot_interfaces REQUIRED)
find_package(voice_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/client_behaviors/cb_abort_all_processes.cpp
  src/client_behaviors/cb_abort_stage1_notice.cpp
  src/client_behaviors/cb_ask_pass_bag_dialog.cpp
  src/client_behaviors/cb_arm_rest.cpp
  src/client_behaviors/cb_execute_grasp.cpp
  src/client_behaviors/cb_ready_follow.cpp
  src/client_behaviors/cb_return_home_managed.cpp
  src/client_behaviors/cb_start_person_follow_session.cpp
  src/client_behaviors/cb_start_unmapped_person_follow_session.cpp
  src/client_behaviors/cb_start_person_follow_stack_managed.cpp
  src/client_behaviors/cb_start_full_system_bringup.cpp
  src/client_behaviors/cb_navigate_to_waypoint.cpp
  src/client_behaviors/cb_wait_5_seconds.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC include)

ament_target_dependencies(
  ${PROJECT_NAME}
  controller_manager_msgs
  geometry_msgs
  nav2_msgs
  putrabot_interfaces
  rclcpp
  rclcpp_action
  smacc2
  std_msgs
  std_srvs
  voice_interfaces
  yaml-cpp
  ament_index_cpp
)

target_link_libraries(${PROJECT_NAME}
  Boost::thread
  yaml-cpp
)

add_executable(waypoint_store_node
  scripts/waypoint_store_node.cpp
)
target_compile_features(waypoint_store_node PUBLIC cxx_std_17)
ament_target_dependencies(
	  waypoint_store_node
	geometry_msgs
  putrabot_interfaces
  rclcpp
  )

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS waypoint_store_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/grasp_ready_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
