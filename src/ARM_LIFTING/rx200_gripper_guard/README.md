# RX200 Gripper Guard

This package is the sole PWM owner during a guarded grasp. The Interbotix `xs_sdk` remains the only DYNAMIXEL bus owner.

```text
xs_sdk → /rx200/sdk_joint_states → gripper guard
xs_sdk ← /rx200/commands/joint_single ← gripper guard
grasp_node → CloseUntilContact action → gripper guard
```

The guard exposes `/rx200/gripper_guard/close_until_contact`, publishes diagnostics on `/rx200/gripper_guard/diagnostics`, and publishes informational contact state on `/rx200/gripper_guard/contact`.

## Calibration required

The following parameters intentionally have neutral defaults and must be measured on the physical gripper before production use:

- `contact.velocity_threshold`
- `contact.position_span_threshold`
- `contact.load_delta_threshold` when load confirmation is enabled
- `contact.aperture_margin`
- `calibration.empty_closed_aperture`
- `contact.required_samples` and `contact.history_length`

Run the read-only logger with:

```bash
ros2 run rx200_gripper_guard gripper_calibration_logger --ros-args \
  -p output_csv:=/tmp/rx200_gripper_calibration.csv
```

Do not calibrate initial contact behavior against a human hand.
