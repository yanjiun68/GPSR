cmake_minimum_required(VERSION 3.5)
project(interbotix_ros_xseries)
find_package(ament_cmake REQUIRED)

if(FALSE) # Section: Tests disabled for production-only workspace builds.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
