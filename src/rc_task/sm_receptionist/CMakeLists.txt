cmake_minimum_required(VERSION 3.8)
project(sm_receptionist)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(geometry_msgs REQUIRED)
find_package(llm_interfaces REQUIRED)
find_package(robot_ai_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(robot_ai_interfaces REQUIRED)
find_package(smacc2 REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sm_receptionist_node
  src/sm_receptionist_node.cpp
)
target_compile_features(sm_receptionist_node PUBLIC cxx_std_17)
target_include_directories(sm_receptionist_node PUBLIC include)
ament_target_dependencies(
  sm_receptionist_node
  action_msgs
  geometry_msgs
  llm_interfaces
  rclcpp
  rclcpp_action
  robot_ai_interfaces
  smacc2
  std_msgs
)
target_link_libraries(sm_receptionist_node
  Boost::thread
)

add_executable(task_receptionist_smacc2_node
  src/task_receptionist_smacc2_node.cpp
)
target_compile_features(task_receptionist_smacc2_node PUBLIC cxx_std_17)
target_include_directories(task_receptionist_smacc2_node PUBLIC include)
ament_target_dependencies(
  task_receptionist_smacc2_node
  robot_ai_interfaces
  nav2_msgs
  rclcpp
  rclcpp_action
  robot_ai_interfaces
  smacc2
  std_msgs
)
target_link_libraries(task_receptionist_smacc2_node
  Boost::thread
  nlohmann_json::nlohmann_json
)

add_executable(sm_receptionist_lifecycle_node
  src/sm_receptionist_lifecycle_node.cpp
)
target_compile_features(sm_receptionist_lifecycle_node PUBLIC cxx_std_17)
target_include_directories(sm_receptionist_lifecycle_node PUBLIC include)
ament_target_dependencies(
  sm_receptionist_lifecycle_node
  nav2_util
  rclcpp
  rclcpp_lifecycle
)

add_executable(task_receptionist_lifecycle_node
  src/task_receptionist_lifecycle_node.cpp
)
target_compile_features(task_receptionist_lifecycle_node PUBLIC cxx_std_17)
target_include_directories(task_receptionist_lifecycle_node PUBLIC include)
ament_target_dependencies(
  task_receptionist_lifecycle_node
  ament_index_cpp
  nav2_util
  rclcpp
  rclcpp_lifecycle
)

install(TARGETS
  sm_receptionist_node
  sm_receptionist_lifecycle_node
  task_receptionist_smacc2_node
  task_receptionist_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
