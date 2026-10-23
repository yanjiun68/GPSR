# STS camera-head MoveIt bringup

The launch uses the full `my_car_rx200` planning model, but only the two camera
head joints are controlled. It never starts Gazebo or `joint_state_publisher_gui`.

Build and test without motors first:

```bash
colcon build --symlink-install --packages-select \
  sts_hardware_interface robot_description head_moveit_config
source install/setup.bash
ros2 launch head_moveit_config head_moveit.launch.py use_mock:=true
```

In RViz choose the `camera_head` planning group and one of the named joint
states (`Center`, `Left`, `Right`, `Up`, `Down`). A two-axis head has no general
6-DOF IK solution, so use joint/named targets rather than an end-effector pose.

After checking centers, signs, limits, and emergency stop, use real hardware:

```bash
ros2 launch head_moveit_config head_moveit.launch.py \
  use_mock:=false serial_port:=/dev/ttyACM0 \
  yaw_motor_id:=1 pitch_motor_id:=2 \
  yaw_center_steps:=2048 pitch_center_steps:=2048 \
  yaw_direction:=1 pitch_direction:=-1
```

Controller action and feedback:

```text
/head/motor_controller/follow_joint_trajectory
/head/joint_states
/head/dynamic_joint_states
```

`joint_state_publisher` merges the raw head feedback into `/joint_states` and
supplies zero defaults for otherwise-uncontrolled joints in this standalone
bringup. When arm or base controllers are also running, include their raw
topics so MoveIt receives the complete measured state instead of defaults:

```bash
ros2 launch head_moveit_config head_moveit.launch.py use_mock:=false \
  joint_state_sources:=/head/joint_states,/rx200/joint_states,/base/joint_states
```

Do not include `/joint_states` itself in `joint_state_sources`; that would make
the aggregator subscribe to its own output.

The initial limits are yaw `[-0.7, 0.7]` rad, pitch `[-0.6, 0.6]` rad, and
maximum speed `0.5 rad/s`. `direction_multiplier` is applied symmetrically to
hardware commands and feedback. Confirm the physical pitch direction with a
small command before permitting full travel.
