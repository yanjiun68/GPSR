import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
import time


class JointStateMerger(Node):
    """
    Merges joint states from arm and lift into a single /joint_states topic.

    Features:
    - Detects stale data and logs warnings
    - Configurable stale threshold
    - Handles missing data gracefully
    """

    def __init__(self):
        super().__init__('joint_state_merger')

        # Parameters
        self.declare_parameter('arm_topic', '/arm/joint_states')
        self.declare_parameter('lift_topic', '/lift/joint_state')
        self.declare_parameter('out_topic', '/joint_states')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('stale_threshold_s', 0.5)  # Warn if data older than this
        self.declare_parameter('warn_throttle_s', 5.0)  # Throttle warnings
        self.declare_parameter('inject_default_lift_joint', True)
        self.declare_parameter('default_lift_joint_name', 'lift_joint')
        self.declare_parameter('default_lift_position', 0.0)

        self.arm_topic = self.get_parameter('arm_topic').value
        self.lift_topic = self.get_parameter('lift_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.stale_threshold = self.get_parameter('stale_threshold_s').value
        self.warn_throttle = self.get_parameter('warn_throttle_s').value
        self.inject_default_lift_joint = self.get_parameter('inject_default_lift_joint').value
        self.default_lift_joint_name = self.get_parameter('default_lift_joint_name').value
        self.default_lift_position = float(self.get_parameter('default_lift_position').value)

        # Publisher and subscribers
        self.pub = self.create_publisher(JointState, self.out_topic, 20)
        self.sub_arm = self.create_subscription(
            JointState, self.arm_topic, self._arm_cb, qos_profile_sensor_data
        )
        self.sub_lift = self.create_subscription(
            JointState, self.lift_topic, self._lift_cb, qos_profile_sensor_data
        )

        # State
        self.arm_latest = None
        self.lift_latest = None
        self.arm_recv_time = 0.0
        self.lift_recv_time = 0.0
        self.last_arm_warn = 0.0
        self.last_lift_warn = 0.0

        # Timer
        period = 1.0 / max(1.0, self.publish_rate)
        self.timer = self.create_timer(period, self._publish_merged)

        self.get_logger().info(
            f'JointStateMerger: {self.arm_topic} + {self.lift_topic} -> {self.out_topic} '
            f'@ {self.publish_rate:.0f}Hz, stale_threshold={self.stale_threshold}s'
        )

    def _arm_cb(self, msg: JointState):
        self.arm_latest = msg
        self.arm_recv_time = time.monotonic()

    def _lift_cb(self, msg: JointState):
        self.lift_latest = msg
        self.lift_recv_time = time.monotonic()

    def _publish_merged(self):
        if self.arm_latest is None and self.lift_latest is None:
            return

        now = time.monotonic()

        # Check for stale arm data
        if self.arm_latest is not None:
            arm_age = now - self.arm_recv_time
            if arm_age > self.stale_threshold and (now - self.last_arm_warn) > self.warn_throttle:
                self.get_logger().warn(f'Arm joint states stale: {arm_age:.2f}s old')
                self.last_arm_warn = now

        # Check for stale lift data
        if self.lift_latest is not None:
            lift_age = now - self.lift_recv_time
            if lift_age > self.stale_threshold and (now - self.last_lift_warn) > self.warn_throttle:
                self.get_logger().warn(f'Lift joint states stale: {lift_age:.2f}s old')
                self.last_lift_warn = now

        # Build merged message
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()

        name = []
        pos = []
        vel = []
        eff = []

        def append_from(js: JointState):
            nonlocal name, pos, vel, eff
            if js is None:
                return
            for i, n in enumerate(js.name):
                # Filter out dummy shadow joint to prevent MoveIt crashes
                if n.endswith('shoulder_shadow'):
                    continue
                
                # Keep SDK joint names unchanged.
                # 'left_finger' already exists in sdk_joint_states and should remain the
                # source of truth for MoveIt gripper planning.

                if n in name:
                    continue
                
                # Check for NaNs
                p = js.position[i] if i < len(js.position) else 0.0
                v = js.velocity[i] if i < len(js.velocity) else 0.0
                e = js.effort[i] if i < len(js.effort) else 0.0

                if p != p:  # NaN check
                    self.get_logger().warn(f'Found NaN position in joint {n}, replacing with 0.0')
                    p = 0.0
                
                name.append(n)
                pos.append(p)
                vel.append(v)
                eff.append(e)

        append_from(self.arm_latest)
        append_from(self.lift_latest)

        # Keep the base->top lift kinematic chain connected even when lift hardware is disabled.
        if self.inject_default_lift_joint and self.default_lift_joint_name not in name:
            name.append(self.default_lift_joint_name)
            pos.append(self.default_lift_position)
            vel.append(0.0)
            eff.append(0.0)

        # Log once to debug
        if not hasattr(self, '_logged_joints'):
            self.get_logger().info(f'Merged joints: {name}')
            self._logged_joints = True

        # Compute mimic joint: right_finger = -left_finger (if missing)
        if 'left_finger' in name and 'right_finger' not in name:
            lf_idx = name.index('left_finger')
            name.append('right_finger')
            pos.append(-pos[lf_idx])  # mimic with multiplier -1
            vel.append(-vel[lf_idx] if lf_idx < len(vel) else 0.0)
            eff.append(eff[lf_idx] if lf_idx < len(eff) else 0.0)
        
        # Provide 'gripper' joint if missing (not used by MoveIt gripper group,
        # but kept for compatibility with existing tooling).
        if 'gripper' not in name:
            name.append('gripper')
            pos.append(0.0)
            vel.append(0.0)
            eff.append(0.0)

        out.name = name
        out.position = pos
        out.velocity = vel
        out.effort = eff

        self.pub.publish(out)


def main():
    rclpy.init()
    node = JointStateMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
