# dio13_serial_bridge (Arduino Nano D2 input)

ROS 2 Python package that reads an Arduino Nano input on **D2** and republishes it as ROS 2 topics. The package name is retained for compatibility with the existing bringup configuration; its nodes, topics, and serial protocol use the accurate `nano_d2` name.

The supplied sketch configures D2 as `INPUT_PULLUP`. Wire a normally-open switch between **D2 and GND**: released is internally pulled high; pressed/grounded reads low and is the active ROS state.

## Topics
- `/nano_d2_grounded` (`std_msgs/Bool`): `true` only when D2 is connected to GND
- `/nano_d2_state` (`std_msgs/String`): `GROUNDED` or `RELEASED`

## Expected Arduino protocol
```text
NANO_D2=GROUNDED
NANO_D2=RELEASED
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
