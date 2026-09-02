from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .event_model import MonitorEvent
from .event_store import EventStore
from .state_cache import StateCache
from .sources.arm_source import ArmSource
from .sources.context_source import ContextSource
from .sources.diagnostics_source import DiagnosticsSource
from .sources.energy_source import EnergySource
from .sources.graph_source import GraphSource
from .sources.gpsr_source import GpsrSource
from .sources.lifecycle_source import LifecycleSource
from .sources.nav2_source import Nav2Source
from .sources.rosout_source import RosoutSource
from .sources.system_source import SystemSource
from .sources.voice_source import VoiceSource


class MonitorCore(Node):
    def __init__(self) -> None:
        super().__init__('gpsr_sys_ui_core')

        # Core / API parameters.
        self._declare('diagnostics_topic', '/diagnostics')
        self._declare('rosout_topic', '/rosout')
        self._declare('state_topic', '/gpsr_system_monitor/state_json')
        self._declare('event_topic', '/gpsr_system_monitor/event_json')
        self._declare('graph_rate_hz', 1.0)
        self._declare('lifecycle_rate_hz', 1.0)
        self._declare('health_rate_hz', 5.0)
        self._declare('system_rate_hz', 1.0)
        self._declare('publish_state_rate_hz', 5.0)
        self._declare('event_history_size', 10000)
        self._declare('event_dedupe_window_sec', 2.0)
        self._declare('collision_stop_persistent_sec', 5.0)

        # Section: Verified structured GPSR and voice owner interfaces.
        self._declare('gpsr_plan_topic', '/gpsr/task_plan')
        self._declare('gpsr_scheduler_state_topic', '/gpsr/state')
        self._declare('gpsr_queue_status_topic', '/gpsr/action_queue_status_json')
        self._declare('voice_transcript_topic', '/voice/stt/transcript')

        # Component inventory.
        self._declare('expected_nodes', [
            '/gpsr_task_scheduler', '/controller_server', '/planner_server', '/bt_navigator',
            '/amcl', '/local_costmap/local_costmap', '/global_costmap/global_costmap',
        ])
        self._declare('optional_node_patterns', ['/collision_monitor', '/camera/*', '/move_group'])
        self._declare('lifecycle_nodes', [
            '/controller_server', '/planner_server', '/bt_navigator',
            '/local_costmap/local_costmap', '/global_costmap/global_costmap',
            '/collision_monitor', '/gpsr_object_detection_node', '/gpsr_person_detection_node',
        ])

        # Nav2.
        self._declare('collision_state_topic', '/collision_monitor_state')
        self._declare('cmd_vel_pre_topic', '/cmd_vel_smoothed')
        self._declare('cmd_vel_post_topic', '/cmd_vel')
        self._declare('odom_topic', '/odom')
        self._declare('navigate_status_topic', '/navigate_to_pose/_action/status')

        # Arm.
        self._declare('joint_state_topics', ['/rx200/joint_states', '/joint_states'])
        self._declare('arm_joints', ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate', 'gripper'])
        self._declare('arm_feedback_stale_sec', 1.0)

        # Context.
        self._declare('supervisor_state_topic', '/putrabot_bringup/managed_process_status_json')

        # Event-only energy observer (no dedicated TUI page).
        self._declare('battery_topic', '/battery_state')
        self._declare('battery_stale_sec', 2.0)
        self._declare('battery_warn_percent', 20.0)
        self._declare('battery_critical_percent', 10.0)

        # Host telemetry.
        self._declare('nvidia_smi_enabled', True)
        self._declare('cpu_warn_percent', 90.0)
        self._declare('ram_warn_percent', 90.0)
        self._declare('disk_warn_percent', 90.0)

        self.cache = StateCache()
        self.state_pub = self.create_publisher(String, self.p('state_topic'), 10)
        self.event_pub = self.create_publisher(String, self.p('event_topic'), 50)
        self.events = EventStore(
            max_events=int(self.p('event_history_size')),
            dedupe_window_sec=float(self.p('event_dedupe_window_sec')),
            on_event=self._publish_event,
        )

        self.graph = GraphSource(
            self, self.cache, self.events,
            float(self.p('graph_rate_hz')),
            self.p('expected_nodes'), self.p('optional_node_patterns'),
        )
        self.lifecycle = LifecycleSource(
            self, self.cache, self.events,
            self.p('lifecycle_nodes'), float(self.p('lifecycle_rate_hz')),
        )
        self.diagnostics = DiagnosticsSource(self, self.cache, self.events, self.p('diagnostics_topic'))
        self.rosout = RosoutSource(self, self.cache, self.events, self.p('rosout_topic'))
        # Retain the context subscriptions without overriding Node.context, a read-only rclpy property.
        self.context_source = ContextSource(
            self, self.cache, self.events, self.p('supervisor_state_topic'),
        )
        # Section: GPSR and Voice sources remain read-only and tolerate absent startup nodes.
        self.gpsr = GpsrSource(
            self,
            self.cache,
            self.events,
            self.p('gpsr_plan_topic'),
            self.p('gpsr_scheduler_state_topic'),
            self.p('gpsr_queue_status_topic'),
            float(self.p('graph_rate_hz')),
        )
        self.voice = VoiceSource(
            self,
            self.cache,
            self.events,
            self.p('voice_transcript_topic'),
            float(self.p('graph_rate_hz')),
        )
        self.nav2 = Nav2Source(
            self, self.cache, self.events,
            collision_state_topic=self.p('collision_state_topic'),
            cmd_pre_topic=self.p('cmd_vel_pre_topic'),
            cmd_post_topic=self.p('cmd_vel_post_topic'),
            odom_topic=self.p('odom_topic'),
            navigate_status_topic=self.p('navigate_status_topic'),
            persistent_sec=float(self.p('collision_stop_persistent_sec')),
        )
        self.arm = ArmSource(
            self, self.cache, self.events,
            self.p('joint_state_topics'), self.p('arm_joints'), float(self.p('arm_feedback_stale_sec')),
        )
        self.system = SystemSource(
            self, self.cache, self.events, float(self.p('system_rate_hz')),
            nvidia_smi=bool(self.p('nvidia_smi_enabled')),
            cpu_warn=float(self.p('cpu_warn_percent')),
            ram_warn=float(self.p('ram_warn_percent')),
            disk_warn=float(self.p('disk_warn_percent')),
        )
        self.energy = EnergySource(
            self, self.cache, self.events, self.p('battery_topic'),
            float(self.p('battery_stale_sec')), float(self.p('battery_warn_percent')),
            float(self.p('battery_critical_percent')),
        )

        self.publish_timer = self.create_timer(
            1.0 / max(0.1, float(self.p('publish_state_rate_hz'))), self._publish_state
        )
        self.events.emit(MonitorEvent('INFO', 'DIAGNOSTICS', 'MONITOR_STARTED', 'gpsr_sys_ui monitor core started', node=self.get_name()))
        self.get_logger().info('gpsr_sys_ui monitor core is running')

    def _declare(self, name: str, default: Any) -> None:
        self.declare_parameter(name, default)

    def p(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _publish_event(self, event: MonitorEvent) -> None:
        payload = event.to_dict()
        # Human-readable time keeps the terminal table compact while first/last_seen
        # retain epoch timestamps for machine use.
        payload['timestamp'] = time.strftime('%H:%M:%S', time.localtime(event.timestamp))
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'), default=str)
        self.event_pub.publish(msg)

    def _publish_state(self) -> None:
        state = self.cache.snapshot()
        nodes = state.get('nodes', []) if isinstance(state.get('nodes'), list) else []
        lifecycle = state.get('lifecycle', []) if isinstance(state.get('lifecycle'), list) else []
        summary = state.get('summary', {}) if isinstance(state.get('summary'), dict) else {}

        # Compatibility aliases expected by the TUI and future external viewers.
        state['components'] = nodes
        state['node_count'] = summary.get('node_count', len(nodes))
        nav = state.get('nav2', {}) if isinstance(state.get('nav2'), dict) else {}
        nav['cmd_vel_pre_linear'] = nav.get('pre_linear', nav.get('cmd_vel_pre_linear', '-'))
        nav['cmd_vel_post_linear'] = nav.get('post_linear', nav.get('cmd_vel_post_linear', '-'))
        nav['components'] = [
            {'name': row.get('name', '-'), 'health': row.get('health', 'UNKNOWN'),
             'state': row.get('actual', row.get('state', '-')), 'message': row.get('message', '')}
            for row in lifecycle
            if row.get('name') in {'/controller_server', '/planner_server', '/bt_navigator', '/collision_monitor'}
        ]
        state['nav2'] = nav
        state['monitor'] = {
            'package': 'gpsr_sys_ui',
            'version': '0.3.0',
            'node': self.get_name(),
            'timestamp': time.time(),
            'event_count': len(self.events.snapshot()),
        }

        msg = String()
        msg.data = json.dumps(state, separators=(',', ':'), default=str)
        self.state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MonitorCore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
