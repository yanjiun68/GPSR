#!/usr/bin/env python3
"""Process-level orchestrator for the RX200 all-in-one grasp workflow."""

import os
import signal
import subprocess
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen


class AllInOneOrchestrator(Node):
    """Launch each step, switch Step2 pipelines, and call grasp services."""

    def __init__(self) -> None:
        super().__init__('all_in_one_orchestrator')

        self.declare_parameter('use_external_drivers', False)
        self.declare_parameter('use_camera', True)
        self.declare_parameter('use_saved_tf', False)
        self.declare_parameter('show_yolo_window', False)
        self.declare_parameter('show_mediapipe_window', True)
        self.declare_parameter('show_selector', True)
        self.declare_parameter('step2_yolo_delay_s', 8.0)
        self.declare_parameter('step3_delay_s', 15.0)
        self.declare_parameter('ready_after_step3_delay_s', 8.0)
        self.declare_parameter('stop_yolo_after_ready_delay_s', 2.0)
        self.declare_parameter('step2_perception_settle_s', 12.0)
        self.declare_parameter('between_grasp_services_s', 2.0)
        self.declare_parameter('service_timeout_s', 120.0)
        # When launched as the phase 2 bundle of carry_my_luggage_phase1, the
        # arm drivers + MoveIt are already resident, and dropping the bag at
        # the end of the sequence is the wrong behaviour (phase 3 will hand
        # the bag over later).  These two switches let the same launch run in
        # either the standalone demo mode or the integrated-pipeline mode.
        self.declare_parameter('skip_step1', False)
        self.declare_parameter('skip_step3', False)
        self.declare_parameter('skip_rest_at_end', False)
        # skip_step2_preview: bypass step2_yolo11n_mediapipe entirely and go
        # straight to step2_perception.  The preview is a MediaPipe+YOLO
        # debug viewer for "which bag is the user pointing at", which is
        # redundant in the integrated pipeline because bag_point_selection
        # already chose the bag in phase 1 and the robot is now directly
        # in front of the target bag.
        self.declare_parameter('skip_step2_preview', False)

        self.use_external_drivers = self._bool_param('use_external_drivers')
        self.use_camera = self._bool_param('use_camera')
        self.use_saved_tf = self._bool_param('use_saved_tf')
        self.show_yolo_window = self._bool_param('show_yolo_window')
        self.show_mediapipe_window = self._bool_param('show_mediapipe_window')
        self.show_selector = self._bool_param('show_selector')
        self.step2_yolo_delay_s = self._float_param('step2_yolo_delay_s')
        self.step3_delay_s = self._float_param('step3_delay_s')
        self.ready_after_step3_delay_s = self._float_param('ready_after_step3_delay_s')
        self.stop_yolo_after_ready_delay_s = self._float_param('stop_yolo_after_ready_delay_s')
        self.step2_perception_settle_s = self._float_param('step2_perception_settle_s')
        self.between_grasp_services_s = self._float_param('between_grasp_services_s')
        self.service_timeout_s = self._float_param('service_timeout_s')
        self.skip_step1 = self._bool_param('skip_step1')
        self.skip_step3 = self._bool_param('skip_step3')
        self.skip_rest_at_end = self._bool_param('skip_rest_at_end')
        self.skip_step2_preview = self._bool_param('skip_step2_preview')

        self._processes: Dict[str, ManagedProcess] = {}
        self._shutdown_requested = False
        self._switch_in_progress = False

        self.create_service(
            Trigger,
            '/all_in_one/switch_to_perception',
            self._switch_to_perception_service,
        )
        self.get_logger().info(
            'Switch service ready: ros2 service call /all_in_one/switch_to_perception std_srvs/srv/Trigger'
        )

        # Completion signal for integrated pipelines (carry_my_luggage_phase1
        # state machine subscribes here to know when it can /kill phase2).
        # TRANSIENT_LOCAL so late subscribers still pick up the last True.
        self._phase2_completed_pub = self.create_publisher(
            Bool,
            '/phase2/completed',
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def _bool_param(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def run(self) -> bool:
        try:
            # -------- Step 1: arm + MoveIt + lift --------
            if self.skip_step1:
                self.get_logger().info(
                    '[ALL-IN-ONE] skip_step1=true — assuming step1 (arm + MoveIt + lift) '
                    'is already running resident'
                )
            else:
                self.get_logger().info('[ALL-IN-ONE] Starting Step1: robot + lift + MoveIt')
                self._start_launch(
                    'step1',
                    'step1_startup.launch.py',
                    [f'use_external_drivers:={self._ros_bool(self.use_external_drivers)}'],
                )

            self._sleep(self.step2_yolo_delay_s)

            # -------- Step 2a: yolo11n_mediapipe preview (optional) --------
            # In the integrated carry_my_luggage_phase1 pipeline this preview
            # is redundant — bag_point_selection already chose the target bag
            # in phase 1, so MediaPipe hand-pointing in front of the arm camera
            # is not needed.  Skipping saves ~25s and one extra camera restart.
            if self.skip_step2_preview:
                self.get_logger().info(
                    '[ALL-IN-ONE] skip_step2_preview=true — bypassing '
                    'step2_yolo11n_mediapipe; going straight to step2_perception'
                )
            else:
                self.get_logger().info('[ALL-IN-ONE] Starting Step2 YOLO11n + MediaPipe')
                self._start_launch(
                    'step2_yolo11n',
                    'step2_yolo11n_mediapipe.launch.py',
                    [
                        f'use_camera:={self._ros_bool(self.use_camera)}',
                        f'show_yolo_window:={self._ros_bool(self.show_yolo_window)}',
                        f'show_mediapipe_window:={self._ros_bool(self.show_mediapipe_window)}',
                    ],
                )

            self._sleep(self.step3_delay_s)

            # -------- Step 3: grasp_node --------
            if self.skip_step3:
                self.get_logger().info(
                    '[ALL-IN-ONE] skip_step3=true — assuming step3 (grasp_node) is already '
                    'running resident'
                )
            else:
                self.get_logger().info('[ALL-IN-ONE] Starting Step3: grasp node')
                self._start_launch(
                    'step3',
                    'step3_grasp.launch.py',
                    [f'use_saved_tf:={self._ros_bool(self.use_saved_tf)}'],
                )

            self._sleep(self.ready_after_step3_delay_s)

            # -------- Move arm to grasp pose --------
            # (No-op if arm is already at grasp pose from phase 1.)
            if not self._call_trigger('/grasp/execute_ready', 'move arm to ready state'):
                return False

            # -------- Step 2b: perception (YOLO handle detector + auto-lock) --------
            if self.skip_step2_preview:
                # Preview was skipped; launch step2_perception directly.
                self.get_logger().info('[ALL-IN-ONE] Starting Step2 perception pipeline (direct path)')
                self._start_launch(
                    'step2_perception',
                    'step2_perception.launch.py',
                    [
                        f'use_camera:={self._ros_bool(self.use_camera)}',
                        f'use_external_drivers:={self._ros_bool(self.use_external_drivers)}',
                        'use_yolo:=true',
                        f'show_selector:={self._ros_bool(self.show_selector)}',
                    ],
                )
            else:
                self._sleep(self.stop_yolo_after_ready_delay_s)
                switched, switch_msg = self._switch_step2_to_perception()
                if not switched:
                    self.get_logger().error(f'[ALL-IN-ONE] Step2 switch failed: {switch_msg}')
                    return False

            self._sleep(self.step2_perception_settle_s)

            if not self._call_trigger('/grasp/execute', 'execute grasp state'):
                return False

            self._sleep(self.between_grasp_services_s)
            if not self._call_trigger('/grasp/execute_ready', 'return to ready state'):
                return False

            # Arm is back at grasp-ready holding the bag.  Signal completion
            # so the integrated pipeline (state machine) can tear down phase 2.
            self._phase2_completed_pub.publish(Bool(data=True))
            self.get_logger().info(
                '[ALL-IN-ONE] Published /phase2/completed=true (arm holding bag in grasp-ready pose)'
            )

            if self.skip_rest_at_end:
                self.get_logger().info(
                    '[ALL-IN-ONE] skip_rest_at_end=true — NOT calling /grasp/rest (phase 3 '
                    'will open the gripper during handover).  Orchestrator enters idle spin.'
                )
            else:
                self._sleep(self.between_grasp_services_s)
                if not self._call_trigger('/grasp/rest', 'move to rest state'):
                    return False

            self.get_logger().info('[ALL-IN-ONE] Sequence complete. Press Ctrl+C to stop all managed launches.')
            while rclpy.ok() and not self._shutdown_requested:
                rclpy.spin_once(self, timeout_sec=0.5)
            return True
        except KeyboardInterrupt:
            self.get_logger().info('[ALL-IN-ONE] Interrupted by user')
            return False
        finally:
            if self._shutdown_requested:
                self._stop_all()

    def request_shutdown(self) -> None:
        self._shutdown_requested = True

    def _switch_to_perception_service(self, request, response):
        """Service callback to stop Step2 YOLO11n and start Step2 perception."""
        del request
        success, message = self._switch_step2_to_perception()
        response.success = success
        response.message = message
        return response

    def _switch_step2_to_perception(self):
        """Stop the YOLO11n Step2 launch and start the normal perception Step2 launch."""
        if self._switch_in_progress:
            return False, 'Step2 switch is already in progress'

        self._switch_in_progress = True
        try:
            if self._is_launch_running('step2_perception'):
                return True, 'Step2 perception is already running'

            if self._has_launch('step2_yolo11n'):
                self.get_logger().info('[ALL-IN-ONE] Stopping Step2 YOLO11n pipeline before perception switch')
                self._stop_launch('step2_yolo11n')
            else:
                self.get_logger().warn('[ALL-IN-ONE] Step2 YOLO11n pipeline is not running; starting perception anyway')

            self.get_logger().info('[ALL-IN-ONE] Starting Step2 perception pipeline')
            self._start_launch(
                'step2_perception',
                'step2_perception.launch.py',
                [
                    f'use_camera:={self._ros_bool(self.use_camera)}',
                    f'use_external_drivers:={self._ros_bool(self.use_external_drivers)}',
                    'use_yolo:=true',
                    f'show_selector:={self._ros_bool(self.show_selector)}',
                ],
            )
            return True, 'Stopped Step2 YOLO11n and started Step2 perception'
        finally:
            self._switch_in_progress = False

    def _start_launch(self, name: str, launch_file: str, args: Optional[List[str]] = None) -> None:
        if self._is_launch_running(name):
            self.get_logger().warn(f'[ALL-IN-ONE] {name} is already running')
            return
        self._processes.pop(name, None)

        cmd = ['ros2', 'launch', 'rx200_handbag_grasp', launch_file]
        if args:
            cmd.extend(args)

        self.get_logger().info(f'[ALL-IN-ONE] $ {" ".join(cmd)}')
        process = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self._processes[name] = ManagedProcess(name=name, process=process)

    def _has_launch(self, name: str) -> bool:
        managed = self._processes.get(name)
        if managed is None:
            return False
        if managed.process.poll() is None:
            return True
        self._processes.pop(name, None)
        return False

    def _is_launch_running(self, name: str) -> bool:
        return self._has_launch(name)

    def _stop_launch(self, name: str) -> None:
        managed = self._processes.pop(name, None)
        if managed is None:
            self.get_logger().warn(f'[ALL-IN-ONE] {name} is not running')
            return
        self._terminate_process(managed)

    def _stop_all(self) -> None:
        for name in list(self._processes.keys())[::-1]:
            self._stop_launch(name)

    def _terminate_process(self, managed: ManagedProcess) -> None:
        process = managed.process
        if process.poll() is not None:
            return

        self.get_logger().info(f'[ALL-IN-ONE] Sending SIGINT to {managed.name}')
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'[ALL-IN-ONE] {managed.name} did not exit; sending SIGTERM')
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5.0)

    def _call_trigger(self, service_name: str, label: str) -> bool:
        self.get_logger().info(f'[ALL-IN-ONE] Calling {service_name}: {label}')
        client = self.create_client(Trigger, service_name)
        deadline = time.monotonic() + self.service_timeout_s

        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                break
            self.get_logger().info(f'[ALL-IN-ONE] Waiting for {service_name}...')
        else:
            self.get_logger().error(f'[ALL-IN-ONE] Service unavailable: {service_name}')
            return False

        future = client.call_async(Trigger.Request())
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not future.done():
            self.get_logger().error(f'[ALL-IN-ONE] Service timed out: {service_name}')
            return False

        result = future.result()
        if result is None:
            self.get_logger().error(f'[ALL-IN-ONE] Service failed without response: {service_name}')
            return False

        if result.success:
            self.get_logger().info(f'[ALL-IN-ONE] {service_name} succeeded: {result.message}')
            return True

        self.get_logger().error(f'[ALL-IN-ONE] {service_name} failed: {result.message}')
        return False

    def _sleep(self, duration_s: float) -> None:
        end_time = time.monotonic() + max(0.0, duration_s)
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=min(0.2, end_time - time.monotonic()))

    @staticmethod
    def _ros_bool(value: bool) -> str:
        return 'true' if value else 'false'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AllInOneOrchestrator()

    def _signal_handler(signum, frame):
        del signum, frame
        node.request_shutdown()
        rclpy.try_shutdown()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    ok = node.run()
    node.destroy_node()
    rclpy.try_shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
