# dio13_monitor_example

Example ROS 2 Python package that subscribes to `/dio13_state` and generates:
- transition logs
- `/dio13_event` software events
- a simple HIGH-event latch

## Build

```bash
colcon build --packages-select dio13_monitor_example
source install/setup.bash
```

## Run

```bash
ros2 launch dio13_monitor_example dio13_monitor.launch.py
```

## Topics

- Input: `/dio13_state` (`std_msgs/Bool`)
- Output: `/dio13_event` (`std_msgs/String`)
