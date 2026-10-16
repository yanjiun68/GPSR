cmake_minimum_required(VERSION 3.8)
project(robocup_home_2025_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  maps
  worlds
  docs
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/generate.py
  scripts/build_2025_vision_world.py
  scripts/generate_vision_reference_map.py
  scripts/render_furniture_previews.py
  tests/gpsr_2025_integration_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/furniture_details.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
