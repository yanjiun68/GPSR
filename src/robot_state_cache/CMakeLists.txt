cmake_minimum_required(VERSION 3.8)
project(robot_state_cache)

# Section: C++17 provides optional values and the standard synchronization primitives used by the cache.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Section: Resolve only the standard ROS dependencies used by the reference adapter.
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Section: The generic cache remains header-only and independent from ROS message types.
add_library(${PROJECT_NAME}_core INTERFACE)
target_include_directories(${PROJECT_NAME}_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Section: The node library permits integration tests and future composed adapters to reuse the implementation.
add_library(${PROJECT_NAME}_ros src/string_cache_node.cpp)
target_include_directories(${PROJECT_NAME}_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_ros diagnostic_msgs rclcpp std_msgs)

# Section: The executable supplies the normal ROS process entry point.
add_executable(robot_state_cache_node src/string_cache_main.cpp)
target_link_libraries(robot_state_cache_node ${PROJECT_NAME}_ros)
ament_target_dependencies(robot_state_cache_node rclcpp)

# Section: Install public APIs, runtime artifacts, and operator configuration.
install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_ros robot_state_cache_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

# Section: Unit and ROS integration tests are enabled by normal colcon test builds.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_circular_cache test/test_circular_cache.cpp)
  if(TARGET test_circular_cache)
    target_link_libraries(test_circular_cache ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_sample_and_hold test/test_sample_and_hold.cpp)
  if(TARGET test_sample_and_hold)
    target_link_libraries(test_sample_and_hold ${PROJECT_NAME}_ros)
    ament_target_dependencies(test_sample_and_hold diagnostic_msgs rclcpp std_msgs)
  endif()
endif()

# Section: Export the reusable header and node libraries to downstream robot subsystems.
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(diagnostic_msgs rclcpp std_msgs)
ament_package()
