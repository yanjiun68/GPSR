# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(realsense2_camera_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/IMUInfo.msg"
  "msg/Extrinsics.msg"
  "msg/Metadata.msg"
  "msg/RGBD.msg"
)

set(realsense2_camera_msgs_rosidl_args
  ${msg_files}
  "srv/DeviceInfo.srv"
  "srv/CalibConfigRead.srv"
  "srv/CalibConfigWrite.srv"
  "action/TriggeredCalibration.action"
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
)

if(FALSE) # Section: Tests disabled for production-only workspace builds.
  find_package(ament_cmake_cppcheck QUIET)
  if(ament_cmake_cppcheck_FOUND)
    list(APPEND realsense2_camera_msgs_rosidl_args ADD_LINTER_TESTS)
  endif()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${realsense2_camera_msgs_rosidl_args}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
