cmake_minimum_required(VERSION 3.8)
project(gz_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  world
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/generate_robocup2026_world.py
  scripts/check_real_time_factor.py
  scripts/piper_trajectory_action_relay.py
  scripts/rx200_pick_place.py
  scripts/rx200_trajectory_action_relay.py
  scripts/washer_controller.py
  scripts/washer_panel.py
  DESTINATION lib/${PROJECT_NAME}
)

# Opt-in integration checks. They start an isolated headless Gazebo process.
install(PROGRAMS
  test/validate_robocup2026_arena.py
  test/validate_robocup2026_assets.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
