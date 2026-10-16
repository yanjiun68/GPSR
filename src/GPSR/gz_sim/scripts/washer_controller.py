#!/usr/bin/env python3
"""Control the simulated washer through joint targets and physical button contacts."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


class WasherState:
    """Door interlock and timed cycle; all times are simulation seconds."""

    def __init__(self, cycle_seconds=30.0, drum_speed=0.8, open_angle=1.8,
                 door_speed=0.6):
        self.cycle_seconds = cycle_seconds
        self.spin_speed = drum_speed
        self.open_angle = open_angle
        self.door_speed = door_speed
        self.door_setpoint = None
        self.powered = False
        self.phase = 'off'
        self.door_target = 0.0
        self.door_angle = 0.0
        self.drum_velocity = 0.0
        self.remaining = cycle_seconds
        self.feedback_time = None
        self.last_tick = None
        self.message = 'Ready: door closed, laundry inside'

    def fresh(self, now):
        return self.feedback_time is not None and 0 <= now-self.feedback_time < 1.0

    def command(self, command, now):
        command = command.strip().lower()
        if command == 'toggle_power':
            command = 'power_off' if self.powered else 'power_on'
        if command == 'toggle_door':
            command = 'close' if self.door_target > 0.1 else 'open'
        if command == 'start_pause':
            command = 'pause' if self.phase == 'running' else 'start'
        if command == 'power_on':
            self.powered = True
            if self.phase == 'off':
                self.phase = 'idle'
        elif command == 'power_off':
            self.powered = False
            self.phase = 'off'
        elif command in ('open', 'close'):
            if not self.fresh(now):
                self.message = 'Waiting for washer joint feedback'
                return False
            if self.phase == 'running' or abs(self.drum_velocity) > 0.05:
                self.message = 'Door locked: pause and wait for drum to stop'
                return False
            self.door_target = self.open_angle if command == 'open' else 0.0
        elif command == 'start':
            if not self.powered:
                self.message = 'Power on first'
                return False
            if not self.fresh(now) or abs(self.door_angle) > 0.04 or self.door_target != 0:
                self.message = 'Close the door and wait until it is latched'
                return False
            if self.phase in ('idle', 'complete', 'off'):
                self.remaining = self.cycle_seconds
            self.phase = 'running'
        elif command == 'pause':
            if self.phase == 'running':
                self.phase = 'paused'
        elif command == 'reset':
            self.phase = 'idle' if self.powered else 'off'
            self.remaining = self.cycle_seconds
        else:
            self.message = f'Unknown command: {command}'
            return False
        self.message = command
        return True

    def tick(self, now):
        dt = 0 if self.last_tick is None else max(0, now-self.last_tick)
        self.last_tick = now
        if self.fresh(now):
            if self.door_setpoint is None:
                # Start the trajectory at the measured angle, not an assumed zero.
                self.door_setpoint = self.door_angle
            else:
                step = self.door_speed * dt
                error = self.door_target - self.door_setpoint
                self.door_setpoint += max(-step, min(step, error))
        if self.phase == 'running':
            if not self.fresh(now) or abs(self.door_angle) > 0.05:
                self.phase = 'paused'
                self.message = 'Paused: joint feedback missing or door not closed'
            else:
                self.remaining = max(0, self.remaining-dt)
                if self.remaining == 0:
                    self.phase = 'complete'
                    self.message = 'Cycle complete; door can be opened after drum stops'
        return self.spin_speed if self.phase == 'running' else 0.0

    def status(self):
        return {'powered': self.powered, 'phase': self.phase,
                'door_angle': self.door_angle, 'door_target': self.door_target,
                'drum_velocity': self.drum_velocity, 'remaining_seconds': self.remaining,
                'message': self.message}


class WasherController(Node):
    def __init__(self):
        super().__init__('washer_controller')
        params = [self.declare_parameter(name, default).value for name, default in
                  [('cycle_seconds',30.0), ('drum_speed',0.8), ('door_open_angle',1.8),
                   ('door_speed',0.6)]]
        if not all(math.isfinite(v) and v > 0 for v in params) or params[2] > 1.9:
            raise ValueError('Invalid washer cycle, speed or door angle parameters')
        self.state = WasherState(*params)
        self.door = self.create_publisher(Float64, '/washing_machine/door_target', 10)
        self.drum = self.create_publisher(Float64, '/washing_machine/drum_speed', 10)
        self.status_pub = self.create_publisher(String, '/washing_machine/status', 10)
        self.create_subscription(String, '/washing_machine/command', self.on_command, 10)
        self.create_subscription(JointState, '/washing_machine/joint_states', self.on_joints, qos_profile_sensor_data)
        self.buttons = {}
        for name, command in [('power','toggle_power'), ('start','start_pause'), ('door','toggle_door')]:
            self.create_subscription(Contacts, f'/washing_machine/buttons/{name}',
                                     lambda msg, key=name, cmd=command: self.on_button(key,cmd,msg),
                                     qos_profile_sensor_data)
        self.create_timer(0.05, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_command(self, msg):
        self.state.command(msg.data, self.now())

    def on_joints(self, msg):
        joints = {name.split('::')[-1]: i for i, name in enumerate(msg.name)}
        if not all(name in joints for name in ('door_hinge','drum_joint')):
            return
        door, drum = joints['door_hinge'], joints['drum_joint']
        if door >= len(msg.position) or drum >= len(msg.velocity):
            return
        self.state.door_angle = msg.position[door]
        self.state.drum_velocity = msg.velocity[drum]
        self.state.feedback_time = self.now()

    def on_button(self, name, command, msg):
        pressed = bool(msg.contacts)
        was_pressed, last_press = self.buttons.get(name, (False, 0.0))
        now = time.monotonic()
        if pressed and not was_pressed and now-last_press > 0.5:
            self.state.command(command, self.now())
            last_press = now
        self.buttons[name] = (pressed, last_press)

    def update(self):
        speed = self.state.tick(self.now())
        if self.state.door_setpoint is not None:
            self.door.publish(Float64(data=self.state.door_setpoint))
        self.drum.publish(Float64(data=speed))
        self.status_pub.publish(String(data=json.dumps(self.state.status())))


def main():
    rclpy.init()
    node = WasherController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.drum.publish(Float64(data=0.0))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
