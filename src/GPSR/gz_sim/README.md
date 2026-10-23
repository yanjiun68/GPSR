# gz_sim

Gazebo Fortress simulation resources for the RoboCup GPSR arena. The package owns
the arena SDF, model resources, ROS–Gazebo bridges, and washing-machine simulation
nodes. The robot model is loaded from `robot_description`; navigation remains in
`my_robot_nav`.

Build and launch from the workspace root:

```bash
colcon build --symlink-install --packages-select gz_sim
source install/setup.bash
ros2 launch gz_sim arena26.launch.py
```

Short form: `ros2 launch gz_sim arena26.launch.py`.

Useful launch options:

```bash
ros2 launch gz_sim robocup_arena.launch.py headless:=true enable_camera:=false
ros2 launch gz_sim robocup_arena.launch.py start_side:=inside start_distance:=0.6
ros2 launch gz_sim robocup_arena.launch.py world:=/absolute/path/to/world.sdf
ros2 launch gz_sim static_world.launch.py
```

The default robot starts 0.4 m outside the `arena_entrance` frame and faces into
the arena. The launch exposes `door_x`, `door_y`, and `door_yaw` overrides for
worlds without that frame.

## Two-axis camera head

The mast D435i is mounted on the simulated STS yaw/pitch head by default. Disable
the moving head while retaining the original fixed camera mount with:

```bash
ros2 launch gz_sim arena26.launch.py enable_head_gimbal:=false
```

Command a target in radians within yaw `[-0.7, 0.7]` and pitch `[-0.6, 0.6]`:

```bash
ros2 topic pub --once /head/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [head_yaw_joint, head_pitch_joint], points: [{positions: [0.35, 0.20], time_from_start: {sec: 2}}]}'
```

The feedback and TF authority is intentionally singular:

```text
/head/joint_trajectory (ROS)
  -> ROS_TO_GZ bridge -> Gazebo trajectory PID
  -> Gazebo JointStatePublisher -> GZ_TO_ROS /joint_states
  -> robot_state_publisher -> head and camera /tf
```

Do not run `joint_state_publisher_gui` or the serial STS hardware interface with
this simulation. The physical STS reference uses the opposite pitch-axis sign;
calibrate and invert that hardware command before sharing these joint names.

## RX200 MoveIt simulation

Start the RoboCup@Home 2025 Gazebo simulation first:

```bash
ros2 launch robocup_home_2025_sim robocup_home_2025.launch.py
```

Then start the RX200 trajectory relays, MoveIt `move_group`, grasp/place
executor, and RViz in a separate terminal:

```bash
ros2 launch gz_sim rx200_moveit_sim.launch.py
```

`rx200_moveit_sim.launch.py` never starts Gazebo or spawns a robot. Configure
the world, spawn pose, camera, and headless mode on
`robocup_home_2025_sim/robocup_home_2025.launch.py`; for example:

```bash
ros2 launch robocup_home_2025_sim robocup_home_2025.launch.py \
  headless:=true spawn_x:=0.50 spawn_y:=-0.39 spawn_yaw:=0.0
ros2 launch gz_sim rx200_moveit_sim.launch.py use_rviz:=false
```

The launch defaults `rviz_config` to the integrated RX200 and camera-head
profile installed by `gz_sim`. Override it when a custom RViz layout is needed:

```bash
ros2 launch gz_sim rx200_moveit_sim.launch.py \
  rviz_config:=/absolute/path/to/custom.rviz
```

The five-DOF RX200 uses position-only LMA IK. In RViz, select
`interbotix_arm` for the arm or `camera_head` for the camera. The head is a
two-DOF joint-space group, so use the `Center`, `Left`, `Right`, `Up`, or
`Down` named state (or set its joint values) rather than an end-effector pose.
The MoveIt controllers execute through:

```text
/rx200/arm_controller/follow_joint_trajectory
/rx200/gripper_controller/follow_joint_trajectory
/head/motor_controller/follow_joint_trajectory
```

For example, execute the `Left` head state from RViz by selecting planning
group `camera_head`, selecting goal state `Left`, then clicking **Plan &
Execute**. Gazebo publishes the resulting joint feedback on `/joint_states`;
do not run the STS hardware controller at the same time.

The manipulation executor accepts desired end-effector poses. For example:

```bash
ros2 topic pub --once /rx200_manipulation/grasp_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_footprint}, pose: {position: {x: 0.55, y: 0.0, z: 0.65}, orientation: {w: 1.0}}}"

ros2 topic pub --once /rx200_manipulation/place_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_footprint}, pose: {position: {x: 0.45, y: 0.20, z: 0.65}, orientation: {w: 1.0}}}"

ros2 topic echo /rx200_manipulation/status
```

The executor performs open, pre-grasp, grasp, close, PlanningScene attach,
lift, place, detach, and retreat operations. PlanningScene attachment handles
collision planning only. Carrying the corresponding physical Gazebo model
requires a contact or detachable-joint plugin for that object's model.
