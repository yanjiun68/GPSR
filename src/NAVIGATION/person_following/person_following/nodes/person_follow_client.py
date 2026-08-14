#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from putrabot_interfaces.action import PersonFollow
from putrabot_interfaces.srv import (
    ClearPerson,
    GetPersonFollowStatus,
    SelectPerson,
    SetFollowParams,
)


class PersonFollowClient(Node):
    def __init__(self):
        super().__init__('person_follow_client')
        self.action_client = ActionClient(self, PersonFollow, '/person_follow/control')
        self.srv_select = self.create_client(SelectPerson, '/person_follow/select_person')
        self.srv_clear = self.create_client(ClearPerson, '/person_follow/clear_person')
        self.srv_stop = self.create_client(Trigger, '/person_follow/stop')
        self.srv_status = self.create_client(GetPersonFollowStatus, '/person_follow/get_status')
        self.srv_set_params = self.create_client(SetFollowParams, '/person_follow/set_params')

    def wait_services(self):
        self.srv_select.wait_for_service(timeout_sec=2.0)
        self.srv_clear.wait_for_service(timeout_sec=2.0)
        self.srv_stop.wait_for_service(timeout_sec=2.0)
        self.srv_status.wait_for_service(timeout_sec=2.0)
        self.srv_set_params.wait_for_service(timeout_sec=2.0)


def main(args=None):
    parser = argparse.ArgumentParser(description='Person follow client')
    sub = parser.add_subparsers(dest='cmd', required=True)

    p_start = sub.add_parser('start')
    p_start.add_argument('--auto-select', action='store_true', default=False)
    p_start.add_argument('--follow-distance', type=float, default=-1.0)
    p_start.add_argument('--face-hold-sec', type=float, default=-1.0)

    sub.add_parser('stop')

    p_select = sub.add_parser('select')
    p_select.add_argument('--track-id', type=int, required=True)

    sub.add_parser('clear')
    sub.add_parser('status')

    p_set = sub.add_parser('set-params')
    p_set.add_argument('--follow-distance', type=float, default=-1.0)
    p_set.add_argument('--face-hold-sec', type=float, default=-1.0)

    cli_args = parser.parse_args(args=args)

    rclpy.init()
    node = PersonFollowClient()

    try:
        if cli_args.cmd == 'start':
            if not node.action_client.wait_for_server(timeout_sec=2.0):
                print('action server not available')
                return

            goal = PersonFollow.Goal()
            goal.auto_select_if_needed = bool(cli_args.auto_select)
            goal.follow_distance_m = float(cli_args.follow_distance)
            goal.face_hold_sec = float(cli_args.face_hold_sec)

            future = node.action_client.send_goal_async(goal, feedback_callback=lambda fb: print(f'feedback: {fb.feedback}'))
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            if not future.done() or future.result() is None:
                print('failed to send goal')
                return

            goal_handle = future.result()
            if not goal_handle.accepted:
                print('goal rejected')
                return

            print('goal accepted')
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            if result_future.result() is not None:
                print(f'result: {result_future.result().result}')

        elif cli_args.cmd == 'stop':
            node.wait_services()
            req = Trigger.Request()
            fut = node.srv_stop.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
            print(fut.result())

        elif cli_args.cmd == 'select':
            node.wait_services()
            req = SelectPerson.Request()
            req.track_id = int(cli_args.track_id)
            fut = node.srv_select.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
            print(fut.result())

        elif cli_args.cmd == 'clear':
            node.wait_services()
            req = ClearPerson.Request()
            fut = node.srv_clear.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
            print(fut.result())

        elif cli_args.cmd == 'status':
            node.wait_services()
            req = GetPersonFollowStatus.Request()
            fut = node.srv_status.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
            print(fut.result())

        elif cli_args.cmd == 'set-params':
            node.wait_services()
            req = SetFollowParams.Request()
            req.follow_distance_m = float(cli_args.follow_distance)
            req.face_hold_sec = float(cli_args.face_hold_sec)
            fut = node.srv_set_params.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
            print(fut.result())

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
