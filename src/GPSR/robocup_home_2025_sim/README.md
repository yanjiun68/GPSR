# RoboCup Malaysia 2025 @Home vision-object Gazebo environment

**Status:** The source-level world and asset checks pass. The vision extension
uses the 20 Gazebo object models supplied in
`RoboCup_MY2025_GPSR_URDF_Gazebo_Objects.zip`; the supplied maps are not SLAM
output.

## Provenance
- Uploaded PDF is the **2024** RoboCup@Home rulebook (last revision 2024-07-09), despite its filename containing 2025.
- 2025 RoboCup Malaysia repository: https://github.com/robocupmalaysia/at-home-2025/tree/main/maps contains `gpsr_layout.png`, `map_layout.png`, and task-specific floor plans.
- This training layout is an engineering reconstruction of the supplied annotated floor plan, not CAD data.

## Layout — revised Home25 plan
- 6.80 m × 6.30 m exterior footprint with 0.25 m exterior walls and 1.6 m wall height. The occupancy maps retain the existing 14 m × 11 m raster extent for Nav2 compatibility.
- Bedroom (NW), kitchen/dining (NE), living/entrance (SW), and office (SE).
- The 0.90 m west entrance is at `(-3.40, -0.51)` and the 0.90 m south exit is at `(-0.58, -3.15)`; the default robot spawn is 0.4 m outside the west entrance, facing east.
- The GPSR instruction point is `(-2.06, -0.72)`, marked by a non-colliding red floor marker.
- Furniture and all 20 supplied vision objects are positioned on the revised plan. Static collision geometry remains deliberately simple for Nav2.

Regenerate the revised layout after editing its sources:

```bash
python3 scripts/generate.py
python3 scripts/build_2025_vision_world.py --world-only
python3 scripts/generate_vision_reference_map.py
```

## Requirements
Gazebo Fortress with ROS 2 Humble, `ros_gz_bridge`, `slam_toolbox`,
`nav2_map_server`, and the workspace's robot packages. The robot must provide a
simulated planar LiDAR on `/scan`, odometry, and TF `odom -> base_link`.

## Start world
```bash
# Build and source the workspace, then launch the world with this project's robot:
ros2 launch robocup_home_2025_sim home25.launch.py

# Optional headless launch with RGB-D rendering disabled for faster SLAM:
ros2 launch robocup_home_2025_sim home25.launch.py \
  headless:=true enable_camera:=false
```

The package launch delegates robot spawning, bridges, state publishing, and the
corrected RGB-D point cloud to `gz_sim/robocup_arena.launch.py`. By default the
robot spawns 0.4 m outside the west entrance and faces east into the arena. The
default world is `worlds/robocup_home_2025_vision.sdf`; the original furniture-
only world remains available as `robocup_home_training.sdf`.

For separate, restartable localization and navigation, start the world first
and then run the dedicated `my_robot_nav` launch:

```bash
ros2 launch my_robot_nav home25_nav_all.launch.py
```

The shorter navigation command is `ros2 launch my_robot_nav home25.launch.py`; the
explicit form is `home25_nav_all.launch.py`. For the
2026 arena world, use `ros2 launch gz_sim arena26.launch.py` (equivalent to
`robocup_arena.launch.py`).

It uses `maps/reference_lidar_map.yaml`, starts map server/AMCL and Nav2, and
leaves Gazebo independent so the navigation driver can be restarted without
resetting the world. Pass `start_sim:=true` to start the world in the same
launch. The Gazebo bridge is the simulated base interface; do not start the
serial hardware `base_driver.py` for this launch.

`home25_nav_all.launch.py` also starts MoveIt, the Gazebo Piper trajectory
relay, eye-in-hand perception, and the GPSR pick/place action adapter by
default. Use one command for the complete simulated stack:

```bash
ros2 launch my_robot_nav home25_nav_all.launch.py \
  start_sim:=true enable_camera:=true use_rviz:=true
```

Disable manipulation with `start_moveit:=false start_pick_place:=false`, or
open a separate MoveIt view with `moveit_rviz:=true`. The public manipulation
actions are `/manipulation/pick_object` and `/manipulation/place_object`.
Place uses the selected object's calibrated AGX recipe; the adapter rejects an
arbitrary non-empty `target_pose` instead of moving to an uncalibrated pose.

To open the vision world independently, expose its model directory first:

```bash
export IGN_GAZEBO_RESOURCE_PATH="$(pwd)/worlds/models:$(pwd)/worlds:${IGN_GAZEBO_RESOURCE_PATH}"
ign gazebo -r worlds/robocup_home_2025_vision.sdf
```

## Vision object assets

The package includes 20 separate models covering drinks, food, snacks, and
tableware. Their catalog and world poses are stored in
`config/robocup_malaysia_2025_objects.json` and
`config/2025_item_poses.csv`. The supplied SDF and STL assets are under
`worlds/models/robocup25_*`. Superseded procedural models are not used or
installed.

The v2 world also adds procedural detail to the static furniture: bed linens
and pillows, sofa cushions, tables and chairs, cabinet faces, appliance doors,
handles, controls, sink fixtures, and storage components. These visuals retain
the original conservative collision proxies for navigation safety. Furniture
details can be regenerated with `scripts/furniture_details.py` through the
`build_2025_vision_world.py` pipeline.

## Gazebo / ROS 2 bridging
Use your robot's bridge configuration if it already provides ROS `/scan`, `/clock`, odometry and TF. For a common gz-sim LiDAR transport topic named `/scan`:
```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
```
Exact Gazebo LiDAR topic names differ by robot and SDF plugin. Confirm with `gz topic -l` and `gz topic -e -t <topic>` before bridging. Bridge odometry or use the existing controller odometry from your robot package; a scan and clock bridge by themselves are insufficient for SLAM. Use `use_sim_time:=true` across localization nodes.

## Make a genuine SLAM-created map
**Not executed in this artifact environment.** With robot spawned, moving, TF and `/scan` operational:
```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_link
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```
Drive the robot through every room at moderate speed without colliding with furniture. After establishing loop closure:
```bash
mkdir -p maps/recorded
ros2 run nav2_map_server map_saver_cli -f maps/recorded/robocup_slam --ros-args -p use_sim_time:=true
```
The output `maps/recorded/robocup_slam.yaml` + `robocup_slam.pgm` is the **real SLAM-derived** result. If `map_saver_cli` does not find `/map`, investigate node lifecycle, QoS, and topic names.

For localization using *the provided synthetic reference map* instead of mapping, load:
```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="$(pwd)/maps/reference_lidar_map.yaml" -p use_sim_time:=true
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```
Then run your existing Nav2 AMCL stack with `/map`, `/scan`, and odometry TF; set an RViz initial pose. Avoid starting `slam_toolbox` mapping and AMCL as conflicting authorities for `map -> odom` simultaneously.

The vision world has a separate geometric reference map:
`maps/vision_geometric_reference.yaml`.

## Map semantics
`maps/reference_lidar_map.yaml`: resolution `0.05` m/cell, occupancy image 280x220 px; world/map origin `[-7.0, -5.5, 0.0]`, ROS conventional PGM (top row = top of map). Dark = occupied, white = free, gray = unobserved. Cells produced by synthetic line-of-sight raycasting from predefined route sampling points; not a perfect ground-truth raster and not robot-collected SLAM.

## Next steps
1. Spawn user robot and verify it is upright and LiDAR scan is correctly oriented.
2. Verify `/clock`, `/scan`, `/odom`, `/tf` and `/tf_static` and stable odom -> base transform.
3. Exercise slam_toolbox and save a true occupancy map.
4. Compare live map to reference map for drift, missing walls and frame discrepancies.
5. Optionally trace official 2025 GPSR layout after supplying a scale reference / dimensioned floor plan.
