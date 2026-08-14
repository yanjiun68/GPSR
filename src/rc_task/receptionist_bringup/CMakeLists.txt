cmake_minimum_required(VERSION 3.8)
project(receptionist_bringup)

# Section: find the ament build system used by this launch-only package.
find_package(ament_cmake REQUIRED)

# Section: install launch files so ros2 launch can discover this package.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Section: export this package to the ROS 2 ament index.
ament_package()
