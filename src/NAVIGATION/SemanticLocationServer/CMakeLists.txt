cmake_minimum_required(VERSION 3.8)
project(semantic_location_server)

# Dependency section: locate Python installation helpers and ROS interface generators.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Interface section: generate a structured topic message and matching query service.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticLocation.msg"
  "srv/GetSemanticLocation.srv"
  DEPENDENCIES builtin_interfaces
)

# Python section: install the reusable classifier and ROS node module.
ament_python_install_package(SemanticLocationServer)

# Executable section: expose the lowercase ROS executable expected by launch files.
install(PROGRAMS
  scripts/semantic_location_server
  DESTINATION lib/${PROJECT_NAME}
)

# Data section: install only infrastructure owned by this standalone package.
install(DIRECTORY
  config
  docs
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

# Test section: run the deterministic geometry and state-tracking unit suite.
if(BUILD_TESTING)
  ament_add_pytest_test(test_semantic_model test/test_semantic_model.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

# Package section: export runtime dependencies and complete package registration.
ament_export_dependencies(rosidl_default_runtime)
ament_package()
