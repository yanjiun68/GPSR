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
