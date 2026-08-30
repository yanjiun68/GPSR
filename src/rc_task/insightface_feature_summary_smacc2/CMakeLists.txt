cmake_minimum_required(VERSION 3.8)
project(insightface_feature_summary_smacc2)

# Build warnings kept consistent with the existing ROS 2 packages in this workspace.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core build system and package lookup dependencies.
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Lifecycle wrapper executable that starts and supervises the YASMIN process.
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
  feature_summary_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the Python YASMIN state machine as a ROS 2 executable.
install(PROGRAMS
  scripts/feature_summary_yasmin_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME feature_summary_yasmin_node
)

# Install public headers used by both executables.
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files for ros2 launch discovery.
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
