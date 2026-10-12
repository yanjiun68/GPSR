# dio13_monitor_example (Arduino Nano D2 input)

Example ROS 2 Python package that subscribes to `/nano_d2_grounded` and generates:
- transition logs
- `/nano_d2_event` software events
- a simple grounded-event latch

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

- Input: `/nano_d2_grounded` (`std_msgs/Bool`; `true` when D2 is grounded)
- Output: `/nano_d2_event` (`std_msgs/String`)
