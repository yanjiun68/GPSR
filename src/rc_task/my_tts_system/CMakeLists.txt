cmake_minimum_required(VERSION 3.8)
project(my_tts_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(my_tts_system_interfaces REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  my_tts_system/guest_text_renderer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME guest_text_renderer
)

install(PROGRAMS
  my_tts_system/json_guest_text_renderer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME json_guest_text_renderer
)

install(PROGRAMS
  my_tts_system/piper_tts_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME piper_tts_node
)

install(PROGRAMS
  my_tts_system/test_guest_profile_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_guest_profile_publisher
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
