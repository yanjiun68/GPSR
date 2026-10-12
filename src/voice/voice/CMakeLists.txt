cmake_minimum_required(VERSION 3.8)
project(voice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(voice_interfaces REQUIRED)

# Install voice model assets used by the Receptionist TTS path.
# Precaution: keep this directory install before running the node from install space.
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install the standalone launch file that starts both observable voice nodes.
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(FALSE) # Section: Tests disabled for production-only workspace builds.
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# Section: Install the shared turn coordinator beside the standalone Python nodes.
install(FILES scripts/audio_turn.py DESTINATION lib/${PROJECT_NAME}) # Make sibling imports work after installation.
install(PROGRAMS
  scripts/receptionist_node.py
  scripts/tts_node.py
  scripts/stt_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
