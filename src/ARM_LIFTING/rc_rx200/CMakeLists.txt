cmake_minimum_required(VERSION 3.8)
project(rc_rx200)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY upstream
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    upstream/rx200_handbag_grasp/scripts/camera_tf_publisher.py
    upstream/rx200_handbag_grasp/scripts/camera_viewer.py
    upstream/rx200_handbag_grasp/scripts/step3_chair_pointing.py
    upstream/rx200_handbag_grasp/scripts/yolo_handle_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES README.md selection_manifest.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
