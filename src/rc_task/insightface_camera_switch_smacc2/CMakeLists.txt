cmake_minimum_required(VERSION 3.8)
project(insightface_camera_switch_smacc2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(nav2_util REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(smacc2 REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(camera_switch_smacc2_node
  src/camera_switch_smacc2_node.cpp
)
target_compile_features(camera_switch_smacc2_node PUBLIC cxx_std_17)
target_include_directories(camera_switch_smacc2_node PUBLIC include)
ament_target_dependencies(
  camera_switch_smacc2_node
  rclcpp
  smacc2
  std_msgs
)
target_link_libraries(camera_switch_smacc2_node
  Boost::thread
  nlohmann_json::nlohmann_json
)

add_executable(camera_switch_lifecycle_node
  src/camera_switch_lifecycle_node.cpp
)
target_compile_features(camera_switch_lifecycle_node PUBLIC cxx_std_17)
target_include_directories(camera_switch_lifecycle_node PUBLIC include)
ament_target_dependencies(
  camera_switch_lifecycle_node
  ament_index_cpp
  nav2_util
  rclcpp
  rclcpp_lifecycle
)

install(TARGETS
  camera_switch_smacc2_node
  camera_switch_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
