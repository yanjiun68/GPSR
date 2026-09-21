cmake_minimum_required(VERSION 3.8)

# Section: Define the ROS 2 package that hosts the device-local Qwen text planner.
project(qwen_vlm_bridge)

# Section: Load ament helpers for installing Python modules and ROS resources.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Section: Install the reusable inference and input-validation Python package.
ament_python_install_package(${PROJECT_NAME})

# Section: Install executable entrypoints with the Qwen virtual-environment launcher intact.
install(PROGRAMS
  scripts/qwen_text_planner.py
  scripts/qwen_text_cli.py
  DESTINATION lib/${PROJECT_NAME}
)

# Section: Install launch and runtime configuration files into the package share directory.
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Section: Install operator documentation beside the runtime configuration.
install(FILES README.md DESTINATION share/${PROJECT_NAME})

# Section: Register focused unit tests that do not load PyTorch or the Qwen model.
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_contract test/test_contract.py)
endif()

# Section: Export this package through the ament index.
ament_package()
