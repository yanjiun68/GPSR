cmake_minimum_required(VERSION 3.8)
project(putrabot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY 
  launch
  configs
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/phase_supervisor.py
  scripts/resolve_serial_roles.py
  DESTINATION lib/${PROJECT_NAME}
)

if(FALSE) # Section: Tests disabled for production-only workspace builds.
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # Section: Exercise generation, resource, readiness, and teardown safety behavior.
  ament_add_pytest_test(test_phase_supervisor test/test_phase_supervisor.py)
endif()

ament_package()
