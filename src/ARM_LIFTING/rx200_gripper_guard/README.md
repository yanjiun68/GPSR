# RX200 Gripper Guard

This package is the sole PWM owner during a guarded grasp. The Interbotix
`xs_sdk` remains the only DYNAMIXEL bus owner.

```text
20 Hz /rx200/sdk_joint_states ──> contact detector ──> stop/contact result
10 Hz Hardware_Error_Status ───> fault latch ───────> PWM 0 / abort
CloseUntilContact action ──────> bounded PWM owner ─> /commands/joint_single
```

The public action remains `/rx200/gripper_guard/close_until_contact`.
Diagnostics are published on `/rx200/gripper_guard/diagnostics`, and the
informational contact flag is published on `/rx200/gripper_guard/contact`.

## Verified command and feedback contract

The handbag Step 1 launch loads
`interbotix_xsarm_control/config/modes.yaml`, whose `gripper` single is in
`pwm` mode. The Interbotix driver therefore interprets `JointSingleCommand.cmd`
as a raw `Goal_PWM` value. Closing is negative, `-75` remains the maximum
configured close magnitude, and PWM `0` is the neutral stop. Before accepting
a hardware-enabled goal, the guard also reads `Operating_Mode` and requires the
DYNAMIXEL PWM value `16`. A missing or mismatched value rejects the goal; a
mode change during closing aborts it. The guard never changes mode, reboots the
servo, or re-enables torque.

The configured SDK feedback rate is 20 Hz. Motor position is radians, motor
velocity is rad/s, and driver effort is estimated motor effort/load. The
`left_finger` position is a single-finger displacement in metres—not total jaw
separation. The RX200 model's closed finger limit is `0.015 m`; this is the
traceable initial `calibration.empty_closed_aperture` value.

## Contact and empty-close parameters

- `calibration.empty_closed_aperture` is the repeatable empty-closed
  left-finger displacement in metres. Initial value: the RX200 URDF lower limit,
  `0.015 m`.
- `contact.aperture_margin` is a positive metre tolerance above empty closure.
  A stalled finger at or below `empty_closed_aperture + aperture_margin` is
  `EMPTY_CLOSED`, not object contact.
- `contact.velocity_threshold` is the maximum absolute gripper motor velocity
  in rad/s for one low-motion sample.
- `contact.position_span_threshold` is the maximum gripper motor progress in
  radians between consecutive 20 Hz samples.
- `contact.required_samples` is the number of consecutive low-motion samples.
  The initial value `3` detects a settled contact in about 100–150 ms at 20 Hz,
  including the first position comparison.
- `contact.load_delta_threshold` is an estimated effort/load delta from the
  pre-close median. Load is secondary evidence, not force control. The default
  `require_load_confirmation: false` keeps the guard usable when load is noisy
  or unavailable.
- `safety.feedback_timeout_sec` is `0.25 s`, or about five missing 20 Hz samples.
  Missing velocity, non-finite feedback, or stale feedback stops and aborts.

The positive threshold defaults are conservative software-derived starting
values, not physical calibration results. Do not leave them at exact zero.

## Prioritized health monitoring

- `Hardware_Error_Status`: 10 Hz, safety-critical.
- `Present_Load`, `Present_PWM`, `Operating_Mode`: 2 Hz round-robin.
- `Present_Temperature`, `Present_Input_Voltage`: 0.2 Hz round-robin.

Every non-zero hardware-error bitmask causes `FAULT`, suppresses all later close
PWM, publishes one PWM-zero stop, and aborts the action. No automatic recovery
is attempted. The estimated worst fault-register observation interval is about
100 ms plus the SDK service response time, compared with roughly five seconds
in the previous 1 Hz five-register round-robin.

## Pre-competition calibration

1. Review the software and confirm the runtime `Operating_Mode` read is `16`.
2. Start with no object and use a deliberately slow, supervised close procedure.
3. Record the stable left-finger position near empty closure without repeatedly
   driving the gripper into DYNAMIXEL shutdown.
4. Set `empty_closed_aperture` to the repeatable measurement and choose a small
   positive `aperture_margin` that covers measurement variation.
5. Record normal moving motor velocity and per-sample position noise at 20 Hz.
6. Set the low-motion thresholds just above measured stopped/noise values but
   below normal closing motion.
7. Verify empty closure returns `empty_close`, never `contact_confirmed`.
8. Test contact first with a soft/compliant object. Test tiny rigid objects only
   after the empty-close boundary is validated.

The read-only CSV logger can collect joint-state feedback:

```bash
ros2 run rx200_gripper_guard gripper_calibration_logger --ros-args \
  -p output_csv:=/tmp/rx200_gripper_calibration.csv
```

Do not calibrate initial contact behavior against a human hand and do not use
hardware overload shutdown as the normal contact sensor.

## Read-only fault diagnostics

Run these only with the RX200 SDK already connected. They do not write because
the local `get_motor_registers` service ignores the request `value` for reads.

```bash
# Read immediately after the red LED/fault occurs.
ros2 service call /rx200/get_motor_registers \
  interbotix_xs_msgs/srv/RegisterValues \
  "{cmd_type: 'single', name: 'gripper', reg: 'Hardware_Error_Status', value: 0}"

# Confirm PWM mode; expected value for this guard is 16.
ros2 service call /rx200/get_motor_registers \
  interbotix_xs_msgs/srv/RegisterValues \
  "{cmd_type: 'single', name: 'gripper', reg: 'Operating_Mode', value: 0}"

# Advisory values useful after a fault.
for reg in Present_Load Present_PWM Present_Temperature Present_Input_Voltage Torque_Enable; do
  ros2 service call /rx200/get_motor_registers \
    interbotix_xs_msgs/srv/RegisterValues \
    "{cmd_type: 'single', name: 'gripper', reg: '${reg}', value: 0}"
done
```

For XL430-style hardware-error bitmasks: `1` is voltage, `4` overheating, `8`
encoder, `16` electrical shock, and `32` overload; combinations are bitwise
sums. Report the returned hardware status and operating mode before deliberate
recovery. Never change `Operating_Mode` live as an ad-hoc fault fix.
