cmake_minimum_required(VERSION 3.8)
project(nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ------------------------------------------------------------------
# 🔹 Install launch (and optionally config) directories
# ------------------------------------------------------------------
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)



# If later you add a config/ folder:
# install(
#   DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME}
# )

# ------------------------------------------------------------------
# Tests (optional)
# ------------------------------------------------------------------


if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_package()
