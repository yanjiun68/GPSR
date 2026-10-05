cmake_minimum_required(VERSION 3.8)
project(robot_yasmin_tasks)

# Section: Preserve the workspace compiler baseline while enabling useful diagnostics.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Section: Resolve only dependencies used by the migrated runtime.
find_package(ament_cmake REQUIRED)
# Section: Install the small Python compatibility library still imported by scheduler and vision.
find_package(ament_cmake_python REQUIRED)
find_package(action_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(putrabot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(robot_ai_interfaces REQUIRED)
find_package(robot_ai_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(voice_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yasmin REQUIRED)

# Section: Keep shared queue, blackboard, state, and ROS adapter behavior in one library.
add_library(${PROJECT_NAME}_core
  src/runtime.cpp
  src/clients.cpp
  src/skills.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  action_msgs ament_index_cpp builtin_interfaces geometry_msgs lifecycle_msgs
  nav2_msgs nav_msgs putrabot_interfaces rclcpp rclcpp_action rclcpp_lifecycle
  robot_ai_interfaces robot_ai_interfaces sensor_msgs std_msgs std_srvs
  tf2 tf2_geometry_msgs tf2_ros voice_interfaces yaml_cpp_vendor yasmin
)
target_link_libraries(${PROJECT_NAME}_core yaml-cpp)

# Section: Preserve every setuptools console-script executable name.
set(runtime_executables
  gpsr_executor
  gpsr_lifecycle_manager
  gpsr_voice_command
  count_objects_skill_server
  count_people_skill_server
  find_object_skill_server
  find_person_skill_server
  guide_person_skill_server
  gpsr_guest_memory_node
  find_person_by_attribute_skill_server
  extract_person_info_skill_server
  skill_availability_resource
  count_objects_blackboard_tts_node
  count_people_blackboard_tts_node
)

# Section: Orchestration nodes own queue execution, lifecycle policy, and voice intake.
add_executable(gpsr_executor src/nodes/orchestration/gpsr_executor.cpp)
add_executable(gpsr_lifecycle_manager src/nodes/orchestration/gpsr_lifecycle_manager.cpp)
add_executable(gpsr_voice_command src/nodes/orchestration/gpsr_voice_command.cpp)

# Section: Skill servers expose lifecycle-managed GPSR action interfaces.
add_executable(count_objects_skill_server src/nodes/skill_servers/count_objects_skill_server.cpp)
add_executable(count_people_skill_server src/nodes/skill_servers/count_people_skill_server.cpp)
add_executable(find_object_skill_server src/nodes/skill_servers/find_object_skill_server.cpp)
add_executable(find_person_skill_server src/nodes/skill_servers/find_person_skill_server.cpp)
add_executable(guide_person_skill_server src/nodes/skill_servers/guide_person_skill_server.cpp)
add_executable(find_person_by_attribute_skill_server src/nodes/skill_servers/find_person_by_attribute_skill_server.cpp)
add_executable(extract_person_info_skill_server src/nodes/skill_servers/extract_person_info_skill_server.cpp)

# Section: Support nodes own shared session state and lifecycle availability resources.
add_executable(gpsr_guest_memory_node src/nodes/support/gpsr_guest_memory_node.cpp)
add_executable(skill_availability_resource src/nodes/support/skill_availability_resource.cpp)

# Section: Reporter nodes translate completed blackboard results into spoken feedback.
add_executable(count_objects_blackboard_tts_node src/nodes/reporters/count_objects_blackboard_tts_node.cpp)
add_executable(count_people_blackboard_tts_node src/nodes/reporters/count_people_blackboard_tts_node.cpp)

# Section: Link every public executable to the same compatibility runtime.
foreach(target IN LISTS runtime_executables)
  target_link_libraries(${target} ${PROJECT_NAME}_core)
  ament_target_dependencies(${target}
    action_msgs ament_index_cpp builtin_interfaces cv_bridge geometry_msgs
    lifecycle_msgs nav2_msgs nav_msgs putrabot_interfaces rclcpp rclcpp_action
    rclcpp_lifecycle robot_ai_interfaces robot_ai_interfaces sensor_msgs
    std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros voice_interfaces
    yaml_cpp_vendor yasmin
  )
endforeach()
target_link_libraries(find_object_skill_server ${OpenCV_LIBS})

# Section: Exercise the compiled factory contracts rather than only retained Python code.
if(FALSE) # Section: Tests disabled for production-only workspace builds.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_skill_factory_cpp test/test_skill_factory.cpp)
  if(TARGET test_skill_factory_cpp)
    target_link_libraries(test_skill_factory_cpp ${PROJECT_NAME}_core)
    ament_target_dependencies(test_skill_factory_cpp geometry_msgs yasmin)
  endif()
endif()

# Section: Retain Python compatibility helpers alongside the production C++ FIND_PERSON path.
ament_python_install_package(${PROJECT_NAME})

# Section: Install C++ runtime, public headers, and unchanged package resources.
install(TARGETS ${PROJECT_NAME}_core ${runtime_executables}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp rclcpp_action robot_ai_interfaces yasmin
)
ament_package()
