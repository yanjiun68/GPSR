cmake_minimum_required(VERSION 3.8)
project(voice_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate ROS interfaces for the voice package.
# Precaution: every .action file listed here must exist before running colcon build.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Receptionist.action"
  "action/SpeakText.action"
  "action/TranscribeSpeech.action"
  "srv/SayText.srv"
  "srv/TranscribeAudio.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
