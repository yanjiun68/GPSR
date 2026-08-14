#!/usr/bin/env python3
from typing import List

import rclpy
from rclpy.node import Node

from receptionist_interfaces.msg import GuestAppearance, PersonProfile
from receptionist_interfaces.srv import (
    CommitCurrentGuest,
    GetProfile,
    GetGuestCount,
    MarkProfileSeated,
    SetCurrentGuestProfile,
    SpeakGuestIntroduction,
)


class PrototypeDemoNode(Node):
    def __init__(self):
        super().__init__("prototype_demo_node")

        self.declare_parameter("target_guest_count", 2)
        self.declare_parameter("guest_examples", [""])

        self.set_current_guest_client = self.create_client(
            SetCurrentGuestProfile, "/guest_memory/set_current_guest_profile"
        )
        self.commit_current_guest_client = self.create_client(
            CommitCurrentGuest, "/guest_memory/commit_current_guest"
        )
        self.get_guest_count_client = self.create_client(
            GetGuestCount, "/guest_memory/get_guest_count"
        )
        self.get_profile_client = self.create_client(
            GetProfile, "/guest_memory/get_profile"
        )
        self.mark_profile_seated_client = self.create_client(
            MarkProfileSeated, "/guest_memory/mark_profile_seated"
        )
        self.speak_guest_introduction_client = self.create_client(
            SpeakGuestIntroduction, "/introduction/speak_guest_introduction"
        )

    def _wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {service_name}...")

    def _call_service(self, client, request, service_name):
        self._wait_for_service(client, service_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            raise RuntimeError(f"{service_name} returned no response.")
        return result

    def _parse_guest_line(self, line: str) -> PersonProfile:
        tokens = [token.strip() for token in line.split(",")]
        if len(tokens) != 5:
            raise ValueError(
                "Expected guest input in the form: name, age, gender, favorite_drink, shirt_color"
            )

        profile = PersonProfile()
        profile.profile_id = "current_guest"
        profile.role = "guest"
        profile.name = tokens[0]
        profile.age = int(tokens[1])
        profile.gender = tokens[2]
        profile.favorite_drink = tokens[3]
        profile.hometown = ""
        profile.is_confirmed = bool(profile.name)
        profile.confidence_score = 1.0
        profile.stamp = self.get_clock().now().to_msg()

        appearance = GuestAppearance()
        appearance.shirt_color = tokens[4]
        appearance.shirt_color_confidence = 1.0
        appearance.stamp = profile.stamp
        profile.appearance = appearance
        return profile

    def _read_guest_line(self, guest_index: int, examples: List[str]) -> str:
        if guest_index - 1 < len(examples):
            line = examples[guest_index - 1]
            self.get_logger().info(f"Using self-defined example {guest_index}: {line}")
            return line

        prompt = (
            f"Enter guest {guest_index} as: name, age, gender, favorite_drink, shirt_color\n> "
        )
        return input(prompt)

    def _store_current_guest(self, profile: PersonProfile):
        request = SetCurrentGuestProfile.Request()
        request.profile = profile
        result = self._call_service(
            self.set_current_guest_client,
            request,
            "/guest_memory/set_current_guest_profile",
        )
        if not result.success:
            raise RuntimeError(result.message)

    def _get_committed_guest_ids(self) -> List[str]:
        request = GetGuestCount.Request()
        result = self._call_service(
            self.get_guest_count_client,
            request,
            "/guest_memory/get_guest_count",
        )
        if not result.success:
            raise RuntimeError(result.message)
        return list(result.committed_guest_ids)

    def _get_profile(self, profile_id: str) -> PersonProfile:
        request = GetProfile.Request()
        request.profile_id = profile_id
        result = self._call_service(
            self.get_profile_client,
            request,
            "/guest_memory/get_profile",
        )
        if not result.success:
            raise RuntimeError(result.message)
        return result.profile

    def _request_introduction(self, target_ids: List[str], include_bonus: bool):
        request = SpeakGuestIntroduction.Request()
        request.source_profile_id = "current_guest"
        request.target_profile_ids = target_ids
        request.include_description_bonus = include_bonus
        result = self._call_service(
            self.speak_guest_introduction_client,
            request,
            "/introduction/speak_guest_introduction",
        )
        if not result.success:
            raise RuntimeError(result.message)
        return list(result.spoken_lines)

    def _commit_current_guest(self, guest_index: int):
        request = CommitCurrentGuest.Request()
        request.new_profile_id = f"guest_{guest_index}"
        result = self._call_service(
            self.commit_current_guest_client,
            request,
            "/guest_memory/commit_current_guest",
        )
        if not result.success:
            raise RuntimeError(result.message)
        return list(result.committed_guest_ids)

    def _mark_profile_seated(self, profile_id: str):
        request = MarkProfileSeated.Request()
        request.profile_id = profile_id
        result = self._call_service(
            self.mark_profile_seated_client,
            request,
            "/guest_memory/mark_profile_seated",
        )
        if not result.success:
            raise RuntimeError(result.message)

    def run_demo(self):
        target_guest_count = int(self.get_parameter("target_guest_count").value)
        guest_examples = [
            example for example in self.get_parameter("guest_examples").value if example
        ]

        self.get_logger().info(
            f"Starting memory prototype demo for {target_guest_count} guest(s)."
        )

        for guest_index in range(1, target_guest_count + 1):
            line = self._read_guest_line(guest_index, guest_examples)
            profile = self._parse_guest_line(line)
            self._store_current_guest(profile)

            committed_guest_ids = self._get_committed_guest_ids()
            target_ids = ["host", *committed_guest_ids]
            introduction_lines = self._request_introduction(
                target_ids,
                include_bonus=bool(committed_guest_ids),
            )

            self.get_logger().info(
                f"Introduction service returned {len(introduction_lines)} line(s):"
            )
            for line in introduction_lines:
                self.get_logger().info(line)

            committed_guest_ids = self._commit_current_guest(guest_index)
            self.get_logger().info(
                f"Committed guest_{guest_index}. Known guests: {committed_guest_ids}"
            )

            committed_profile = self._get_profile(f"guest_{guest_index}")
            self.get_logger().info(
                "Fetched committed profile "
                f"{committed_profile.profile_id} for {committed_profile.name}."
            )

            self._mark_profile_seated(f"guest_{guest_index}")
            seated_profile = self._get_profile(f"guest_{guest_index}")
            self.get_logger().info(
                "Moved profile to seated memory "
                f"for {seated_profile.name or seated_profile.profile_id}."
            )

        self.get_logger().info("Memory prototype demo finished successfully.")


def main(args=None):
    rclpy.init(args=args)
    node = PrototypeDemoNode()

    exit_code = 0
    try:
        node.run_demo()
    except Exception as exc:  # pragma: no cover - surfaced through process exit
        node.get_logger().error(str(exc))
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
