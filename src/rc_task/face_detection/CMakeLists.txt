cmake_minimum_required(VERSION 3.8)
project(face_detection)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(receptionist_interfaces REQUIRED)
find_package(putrabot_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

function(install_python_node node_name)
  install(PROGRAMS
    ${PROJECT_NAME}/${node_name}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${node_name}
  )
endfunction()

install_python_node(appearance_receptionist)
install_python_node(tracked_appearance_lifecycle)
install_python_node(detection_receptionist)
install_python_node(face_perception)
install_python_node(guest_database)
install_python_node(profile_capture_receptionist)
install_python_node(profile_identification_receptionist)

install_python_node(detected_items_node)
install_python_node(stable_detected_items_node)

# Section: Install the camera-free Python environment health check as a package utility.
install(PROGRAMS
  scripts/check_python_environment.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME check_python_environment
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Section: Install operator-supplied YuNet/SFace ONNX files and the model placement guide.
install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Section: Install operator documentation and the reproducible Python dependency manifest.
install(FILES
  USAGE.md
  requirements.txt
  requirements-face.txt
  DESTINATION share/${PROJECT_NAME}
)

file(GLOB MODEL_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.pt")
if(MODEL_FILES)
  install(FILES
    ${MODEL_FILES}
    DESTINATION share/${PROJECT_NAME}/models
  )
endif()

install(DIRECTORY
  detection_pt/
  DESTINATION share/${PROJECT_NAME}/detection_pt
  FILES_MATCHING
    PATTERN "*.pt"
    PATTERN "*.png"
    PATTERN "*.py"
)

set(VISION_APPEARANCE_MODELS
  "${CMAKE_CURRENT_SOURCE_DIR}/../../vision/models/glasses.pt"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../vision/models/tshirt.pt"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../vision/models/facce.pt"
)
foreach(VISION_APPEARANCE_MODEL ${VISION_APPEARANCE_MODELS})
if(EXISTS "${VISION_APPEARANCE_MODEL}")
  install(FILES
    "${VISION_APPEARANCE_MODEL}"
    DESTINATION share/${PROJECT_NAME}/detection_pt
  )
  install(FILES
    "${VISION_APPEARANCE_MODEL}"
    DESTINATION share/${PROJECT_NAME}/models
  )
endif()
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # Section: Pure-Python policy tests run without cameras, models, or a ROS graph.
  ament_add_pytest_test(
    test_identity_store
    test/test_identity_store.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  # Section: ROS-independent YuNet/SFace coordinate and normalization policy tests.
  ament_add_pytest_test(
    test_face_backend
    test/test_face_backend.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  # Section: YASMIN integration tests run with fake messages and no live ROS graph.
  ament_add_pytest_test(
    test_yasmin_profile_states
    test/test_yasmin_profile_states.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  # Section: Shared estimator tests verify ROI, HSV, and per-track temporal behavior.
  ament_add_pytest_test(
    test_appearance_estimators
    test/test_appearance_estimators.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
