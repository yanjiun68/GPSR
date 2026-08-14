# dio13_serial_bridge

ROS 2 Python package that reads Arduino serial lines such as `DIO13=HIGH` and `DIO13=LOW` and republishes them as ROS 2 topics.

## Topics
- `/dio13_state` (`std_msgs/Bool`)
- `/dio13_state_text` (`std_msgs/String`)

## Expected Arduino protocol
```text
DIO13=HIGH
DIO13=LOW
```

## Build
```bash
cd ~/your_ws
colcon build --packages-select dio13_serial_bridge
source install/setup.bash
```

## Run
```bash
ros2 launch dio13_serial_bridge dio13_serial_bridge.launch.py port:=/dev/ttyUSB0 baud_rate:=115200
```
