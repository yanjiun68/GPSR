# my_tts_system

ROS 2 Python package with:
- `guest_text_renderer`: consumes individual guest profile topics and sends rendered text to the Piper TTS action server
- `piper_tts_node`: exposes `/tts/speak` and invokes Piper locally for offline speech playback
- `test_guest_profile_publisher`: simple in-memory guest profile store and test publisher

## Topics
- Guest profile value topics:
  - `/guest_profile/target` (`std_msgs/msg/String`)
  - `/guest_profile/guest_name` (`std_msgs/msg/String`)
  - `/guest_profile/age` (`std_msgs/msg/String`)
  - `/guest_profile/favourite_drink` (`std_msgs/msg/String`)
  - `/guest_profile/shirt_color` (`std_msgs/msg/String`)
  - `/guest_profile/item1` (`std_msgs/msg/String`)
  - `/guest_profile/item2` (`std_msgs/msg/String`)
  - `/guest_profile/item3` (`std_msgs/msg/String`)
  - `/guest_profile/item4` (`std_msgs/msg/String`)
- Guest profile render trigger: `/guest_profile/ready` (`std_msgs/msg/Empty`)
- Republish stored profile trigger: `/guest_profile/publish_stored` (`std_msgs/msg/Empty`)
- TTS action: `/tts/speak` (`my_tts_system_interfaces/action/SpeakText`)

## Build
```bash
cd ~/your_ros2_ws
cp -r /path/to/my_tts_system src/
colcon build --packages-select my_tts_system
source install/setup.bash
```

## Run
```bash
ros2 launch my_tts_system tts_pipeline.launch.py
```

Run with the built-in dummy profile publisher for a standalone test:

```bash
ros2 launch my_tts_system tts_pipeline.launch.py use_test_profile_publisher:=true
```

## Note
Update Piper model paths in `config/tts_params.yaml` before running.
