# Incheon 2026 RoboCup@Home arena reconstruction

`robocup2026_incheon.sdf` reproduces the published 2026 room arrangement and
furniture locations. It is a simulation reconstruction, not an organizer-supplied
SDF or a surveyed replica. All furniture is made from local collision/visual
primitives; opening this world does not require Fuel downloads.

## Official references

- [Incheon2026 repository](https://github.com/RoboCupAtHome/Incheon2026)
- [Arena plan](https://github.com/RoboCupAtHome/Incheon2026/blob/993bb7015566b8c943cd379db781257cbd6ff81b/maps/Arena.png)
- [Named furniture](https://github.com/RoboCupAtHome/Incheon2026/blob/993bb7015566b8c943cd379db781257cbd6ff81b/maps/LocationsNames.png)
- [Inspection plan](https://github.com/RoboCupAtHome/Incheon2026/blob/993bb7015566b8c943cd379db781257cbd6ff81b/maps/Inspection.png)
- [2026.1 announcement](https://github.com/RoboCupAtHome/RuleBook/releases/tag/2026.1)

Reference revision: `993bb7015566b8c943cd379db781257cbd6ff81b`.
For 2026, the league is unified; former DSPL teams join the unified OPL.
The official venue diagram shows three similar arenas, not separate OPL/DSPL
floor plans. This world follows the common Arena.png layout.

## Dimensions and assumptions

The published PNGs have no metric dimensions. The simulation uses **10 x 10 m
between exterior wall centre lines**, from the 504 x 504 pixel arena outline.
Thus the traced doors are approximately 1 m wide. This is an editable assumption,
not a claimed official measurement. Interior partitions and furniture footprints
preserve the proportions and relative positions in the diagram.

- Origin: southwest exterior wall centre intersection; X east, Y north, Z up.
- Wall thickness: 0.15 m; height: 1.2 m (simulation assumptions).
- Laundry northwest, bedroom northeast, living room southwest, kitchen southeast.
- The lower west door into the living room is designated the entrance; the upper
  west door into the laundry is designated the exit. The reference diagrams do
  not label these assignments explicitly.
- Entrance centre: `(0, 3.5119048, 0)`; inward heading `0` rad (+X).
- Default robot `base_footprint`: `(-0.4, 3.5119048)`, heading +X. The 0.4 m is
  measured from the door plane to the base centre, not to the front bumper.
- Spawn height: 0.15 m; the robot settles onto its wheels under gravity.
- The entry leaf is static and parked open inward. It is not an actuated door.
- Furniture heights and details are approximations. Appliances are static props;
  cupboards and appliances have no articulated doors. No people or task objects
  are spawned automatically.

The robot comes directly from `robot_description/urdf/sim/my_car.urdf.xacro`; it uses
`robot_description` only to resolve the chassis meshes referenced by that file.

## Build and run (from workspace root)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_description my_robot_nav --executor sequential
source install/setup.bash
ros2 launch gz_sim robocup_arena.launch.py
```

The launch uses `ign gazebo` for this machine's Fortress installation. Its
`gz` executable belongs to Gazebo Classic and does not accept `gz sim`.
The world launches with the robot, robot_state_publisher, and bridges for clock,
scan, velocity commands, odometry, joint states, TF, RGB, depth and camera info.
It does not start real hardware drivers, Nav2, SLAM or autonomous motion.

If launch reports `executable 'washer_controller.py' not found`, check executable
permissions as well as whether the file exists. With `--symlink-install`, the
installed program points to the source script; a non-executable source aborts
the whole launch before the robot can spawn. Restore permissions with:

```bash
chmod +x src/GPSR/gz_sim/scripts/washer_controller.py src/GPSR/gz_sim/scripts/washer_panel.py
```

World only (no ROS build required):

```bash
ign gazebo -r src/GPSR/gz_sim/world/robocup2026_incheon.sdf
```

Server without GUI (camera and LiDAR still need a working render context):

```bash
ros2 launch gz_sim robocup_arena.launch.py headless:=true
```

Override the starting position relative to the door:

```bash
ros2 launch gz_sim robocup_arena.launch.py start_side:=inside
ros2 launch gz_sim robocup_arena.launch.py start_distance:=0.7
```

The entry pose is read from the world's `arena_entrance` frame. Explicit
`door_x`, `door_y`, `door_yaw` arguments can override it. If you edit the entrance
geometry, update this frame as well.

## Run the integration validation

After building and sourcing the workspace:

```bash
ros2 run gz_sim validate_robocup2026_arena.py
```

The source is `test/validate_robocup2026_arena.py`. It launches its own headless
simulation, checks the default robot position 0.4 m outside the entrance,
requires finite LiDAR returns and an RGB frame, and drives forward at 0.15 m/s
for 8 wall-clock seconds to check entrance traversal. Startup waits up to 45
seconds for sensor data. Working GPU/headless rendering is still required.

The test uses ROS domain **184** and a unique Gazebo partition. Keep that ROS
domain free of hardware and other validation runs. It stops its launched process
group on completion or failure and preserves logs in the printed
`/tmp/putrabot-arena2026-test-*/simulation.log` directory.

Success prints `PASS` and exits with status 0; a failed check exits nonzero.
This is an opt-in integration test, not part of `colcon test`. Its expected pose
and traversal thresholds target the default 10 x 10 m arena; update these
expectations if you rescale or relocate the entrance.

## Edit or rescale the arena

`config/robocup2026_arena.json` stores the traced pixel coordinates, assumed
metric width, wall dimensions and furniture heights. Edit `scale.width_metres`
when measured dimensions become available, then regenerate:

```bash
python3 src/GPSR/gz_sim/scripts/generate_robocup2026_world.py \
  --config src/GPSR/gz_sim/config/robocup2026_arena.json \
  --output src/GPSR/gz_sim/world/robocup2026_incheon.sdf
colcon build --packages-select my_robot_nav --executor sequential
```

Regeneration overwrites the generated SDF. Keep repeatable changes in the JSON
or generator. Horizontal footprints scale with arena width; explicit heights and
wall thickness retain their metre values. The entrance frame scales with the
geometry, while the requested 0.4 m robot offset stays fixed.

Do not load the older `robocup_1.pgm` for localization in this arena: its geometry
differs. Map this world with SLAM before configuring Nav2.
