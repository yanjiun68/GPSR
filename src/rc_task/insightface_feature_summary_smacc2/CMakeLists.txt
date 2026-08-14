cmake_minimum_required(VERSION 3.8)
project(insightface_feature_summary_smacc2)

# Build warnings kept consistent with the existing SMACC2 packages in this workspace.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core build system and package lookup dependencies.
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(nav2_util REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(receptionist_interfaces REQUIRED)
find_package(smacc2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(voice_interfaces REQUIRED)

# Main SMACC2 state machine executable.
add_executable(feature_summary_smacc2_node
  src/feature_summary_smacc2_node.cpp
)
target_compile_features(feature_summary_smacc2_node PUBLIC cxx_std_17)
target_include_directories(feature_summary_smacc2_node PUBLIC include)
ament_target_dependencies(
  feature_summary_smacc2_node
  rclcpp
  receptionist_interfaces
  smacc2
  std_msgs
  voice_interfaces
)
target_link_libraries(feature_summary_smacc2_node
  Boost::thread
  nlohmann_json::nlohmann_json
)

# Lifecycle wrapper executable that starts and supervises the SMACC2 process.
add_executable(feature_summary_lifecycle_node
  src/feature_summary_lifecycle_node.cpp
)
target_compile_features(feature_summary_lifecycle_node PUBLIC cxx_std_17)
target_include_directories(feature_summary_lifecycle_node PUBLIC include)
ament_target_dependencies(
  feature_summary_lifecycle_node
  ament_index_cpp
  nav2_util
  rclcpp
  rclcpp_lifecycle
)

# Install runtime executables into the package lib directory.
install(TARGETS
  feature_summary_smacc2_node
  feature_summary_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install public headers used by both executables.
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files for ros2 launch discovery.
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
