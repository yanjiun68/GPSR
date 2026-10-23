#!/usr/bin/env python3
"""MoveIt pick/place sequence for the RX200 Gazebo simulation.

Publish a desired end-effector pose to ``~/grasp_pose`` to open, approach,
grasp, close, attach a planning-scene object, and lift. Publish a desired pose
to ``~/place_pose`` to place, open, detach, and retreat.

The PlanningScene attachment is intentionally separate from Gazebo physics.
For physical object transport, a Gazebo detachable-joint/contact plugin must
consume the same grasp/release events.
"""

import copy
import json
import math
import threading

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    PlanningOptions,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped transforms
import tf2_ros
from trajectory_msgs.msg import JointTrajectoryPoint


def finite_pose(pose):
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
    )
    norm = math.sqrt(sum(value * value for value in values[3:]))
    return all(math.isfinite(value) for value in values) and norm > 1e-6


def rotated_x_axis(quaternion):
    """Return the local +X axis expressed in the quaternion's parent frame."""
    x, y, z, w = (
        quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + z * w),
        2.0 * (x * z - y * w),
    )


class Rx200PickPlace(Node):
    def __init__(self):
        super().__init__('rx200_pick_place')
        self.declare_parameter('planning_frame', 'base_footprint')
        self.declare_parameter('planning_group', 'interbotix_arm')
        self.declare_parameter('end_effector_link', 'rx200/ee_gripper_link')
        self.declare_parameter('approach_distance', 0.08)
        self.declare_parameter('lift_distance', 0.10)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('velocity_scaling', 0.25)
        self.declare_parameter('acceleration_scaling', 0.25)
        self.declare_parameter('open_position', 0.037)
        self.declare_parameter('closed_position', 0.015)
        self.declare_parameter('gripper_duration', 1.0)
        self.declare_parameter('object_id', 'target_object')
        self.declare_parameter('object_dimensions', [0.06, 0.06, 0.12])

        self.planning_frame = self.get_parameter('planning_frame').value
        self.group = self.get_parameter('planning_group').value
        self.ee_link = self.get_parameter('end_effector_link').value
        self.busy_lock = threading.Lock()
        self.busy = False
        self.object_attached = False
        self.last_grasp_pose = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.move_group = ActionClient(self, MoveGroup, '/move_action')
        self.gripper = ActionClient(
            self, FollowJointTrajectory,
            '/rx200/gripper_controller/follow_joint_trajectory')
        self.apply_scene = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        self.status_publisher = self.create_publisher(
            String, '~/status', 10)
        self.create_subscription(
            PoseStamped, '~/grasp_pose', self.on_grasp_pose, 10)
        self.create_subscription(
            PoseStamped, '~/place_pose', self.on_place_pose, 10)
        self.publish_status('idle', 'ready for grasp_pose')

    def publish_status(self, state, detail='', failure_type=''):
        message = String()
        message.data = json.dumps({
            'state': state,
            'detail': detail,
            'failure_type': failure_type,
        })
        self.status_publisher.publish(message)
        log = self.get_logger().error if failure_type else self.get_logger().info
        log(f'{state}: {detail}')

    def begin(self):
        with self.busy_lock:
            if self.busy:
                return False
            self.busy = True
            return True

    def finish(self):
        with self.busy_lock:
            self.busy = False

    def on_grasp_pose(self, message):
        if not self.begin():
            self.publish_status(
                'rejected', 'another manipulation is active', 'lifecycle_skill_unavailable')
            return
        threading.Thread(
            target=self.pick, args=(message,), daemon=True).start()

    def on_place_pose(self, message):
        if not self.begin():
            self.publish_status(
                'rejected', 'another manipulation is active', 'lifecycle_skill_unavailable')
            return
        threading.Thread(
            target=self.place, args=(message,), daemon=True).start()

    @staticmethod
    def wait_future(future, timeout):
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(timeout):
            raise TimeoutError('ROS request timed out')
        return future.result()

    def transform_pose(self, message):
        if not message.header.frame_id or not finite_pose(message.pose):
            raise ValueError('pose requires a frame_id, finite values, and a valid quaternion')
        return self.tf_buffer.transform(
            message, self.planning_frame, timeout=Duration(seconds=2.0))

    def offset_pose(self, source, local_x=0.0, world_z=0.0):
        result = copy.deepcopy(source)
        axis = rotated_x_axis(source.pose.orientation)
        result.pose.position.x += local_x * axis[0]
        result.pose.position.y += local_x * axis[1]
        result.pose.position.z += local_x * axis[2] + world_z
        return result

    def position_goal(self, target):
        constraint = PositionConstraint()
        constraint.header.frame_id = self.planning_frame
        constraint.link_name = self.ee_link
        constraint.weight = 1.0
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [float(self.get_parameter('position_tolerance').value)]
        region_pose = Pose()
        region_pose.position = target.pose.position
        region_pose.orientation.w = 1.0
        constraint.constraint_region.primitives = [region]
        constraint.constraint_region.primitive_poses = [region_pose]
        goals = Constraints()
        goals.name = 'rx200_end_effector_position'
        goals.position_constraints = [constraint]
        return goals

    def plan_and_execute(self, target, label):
        if not self.move_group.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('MoveIt /move_action is unavailable')

        request = MotionPlanRequest()
        request.group_name = self.group
        request.pipeline_id = 'move_group'
        request.num_planning_attempts = 5
        request.allowed_planning_time = float(
            self.get_parameter('planning_time').value)
        request.max_velocity_scaling_factor = float(
            self.get_parameter('velocity_scaling').value)
        request.max_acceleration_scaling_factor = float(
            self.get_parameter('acceleration_scaling').value)
        request.start_state.is_diff = True
        request.goal_constraints = [self.position_goal(target)]
        request.workspace_parameters.header.frame_id = self.planning_frame
        request.workspace_parameters.min_corner.x = -2.0
        request.workspace_parameters.min_corner.y = -2.0
        request.workspace_parameters.min_corner.z = -0.2
        request.workspace_parameters.max_corner.x = 2.0
        request.workspace_parameters.max_corner.y = 2.0
        request.workspace_parameters.max_corner.z = 2.0

        options = PlanningOptions()
        options.plan_only = False
        options.replan = True
        options.replan_attempts = 2
        options.replan_delay = 0.2
        options.planning_scene_diff.is_diff = True
        options.planning_scene_diff.robot_state.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options
        self.publish_status('moving', label)
        goal_handle = self.wait_future(
            self.move_group.send_goal_async(goal), 5.0)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'MoveIt rejected {label} goal')
        wrapped_result = self.wait_future(
            goal_handle.get_result_async(),
            float(self.get_parameter('planning_time').value) + 30.0)
        result = wrapped_result.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f'{label} failed with MoveIt error {result.error_code.val}')

    def command_gripper(self, position, label):
        if not self.gripper.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('RX200 gripper action is unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['left_finger']
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        duration = float(self.get_parameter('gripper_duration').value)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal.trajectory.points = [point]
        self.publish_status('gripper', label)
        goal_handle = self.wait_future(self.gripper.send_goal_async(goal), 5.0)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'gripper rejected {label} goal')
        wrapped_result = self.wait_future(
            goal_handle.get_result_async(), duration + 10.0)
        if wrapped_result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f'gripper {label} failed: {wrapped_result.result.error_string}')

    def update_attachment(self, attach, object_pose=None):
        if not self.apply_scene.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('MoveIt apply_planning_scene service is unavailable')
        object_id = self.get_parameter('object_id').value
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        if attach:
            collision = CollisionObject()
            collision.header.frame_id = self.planning_frame
            collision.id = object_id
            collision.operation = CollisionObject.ADD
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                float(value)
                for value in self.get_parameter('object_dimensions').value
            ]
            collision.primitives = [primitive]
            collision.primitive_poses = [copy.deepcopy(object_pose.pose)]
            attached = AttachedCollisionObject()
            attached.link_name = self.ee_link
            attached.object = collision
            attached.touch_links = [
                self.ee_link,
                'rx200/fingers_link',
                'rx200/left_finger_link',
                'rx200/right_finger_link',
                'rx200/gripper_bar_link',
            ]
            scene.robot_state.attached_collision_objects = [attached]
        else:
            attached = AttachedCollisionObject()
            attached.link_name = self.ee_link
            attached.object.id = object_id
            attached.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects = [attached]
            remove = CollisionObject()
            remove.id = object_id
            remove.operation = CollisionObject.REMOVE
            scene.world.collision_objects = [remove]

        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self.wait_future(
            self.apply_scene.call_async(request), 5.0)
        if response is None or not response.success:
            raise RuntimeError('MoveIt rejected the PlanningScene update')
        self.object_attached = attach

    def pick(self, message):
        try:
            grasp = self.transform_pose(message)
            approach_distance = float(
                self.get_parameter('approach_distance').value)
            lift_distance = float(self.get_parameter('lift_distance').value)
            pregrasp = self.offset_pose(grasp, local_x=-approach_distance)
            lifted = self.offset_pose(grasp, world_z=lift_distance)
            self.publish_status('picking', 'grasp pose accepted')
            self.command_gripper(
                self.get_parameter('open_position').value, 'open')
            self.plan_and_execute(pregrasp, 'pre-grasp')
            self.plan_and_execute(grasp, 'grasp')
            self.command_gripper(
                self.get_parameter('closed_position').value, 'close')
            self.update_attachment(True, grasp)
            self.last_grasp_pose = grasp
            self.plan_and_execute(lifted, 'lift')
            self.publish_status(
                'pick_succeeded',
                'object attached in MoveIt; Gazebo physics attachment requires a detachable-joint plugin')
        except ValueError as error:
            self.publish_status('pick_failed', str(error), 'invalid_input')
        except (tf2_ros.TransformException, TimeoutError) as error:
            self.publish_status('pick_failed', str(error), 'perception_unavailable')
        except RuntimeError as error:
            failure = (
                'lifecycle_skill_unavailable'
                if 'unavailable' in str(error) else 'manipulation_failure')
            self.publish_status('pick_failed', str(error), failure)
        finally:
            self.finish()

    def place(self, message):
        try:
            if not self.object_attached:
                raise ValueError('no object is currently attached')
            place_pose = self.transform_pose(message)
            lift_distance = float(self.get_parameter('lift_distance').value)
            preplace = self.offset_pose(place_pose, world_z=lift_distance)
            self.publish_status('placing', 'place pose accepted')
            self.plan_and_execute(preplace, 'pre-place')
            self.plan_and_execute(place_pose, 'place')
            self.command_gripper(
                self.get_parameter('open_position').value, 'open')
            self.update_attachment(False)
            self.plan_and_execute(preplace, 'retreat')
            self.publish_status('place_succeeded', 'object detached from MoveIt')
        except ValueError as error:
            self.publish_status('place_failed', str(error), 'invalid_input')
        except (tf2_ros.TransformException, TimeoutError) as error:
            self.publish_status('place_failed', str(error), 'perception_unavailable')
        except RuntimeError as error:
            failure = (
                'lifecycle_skill_unavailable'
                if 'unavailable' in str(error) else 'manipulation_failure')
            self.publish_status('place_failed', str(error), failure)
        finally:
            self.finish()


def main(args=None):
    rclpy.init(args=args)
    node = Rx200PickPlace()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
