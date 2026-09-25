#!/usr/bin/env python3
import copy
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from robot_ai_interfaces.msg import GuestAppearance, PersonProfile
from robot_ai_interfaces.srv import (
    ClearCurrentGuest,
    CommitCurrentGuest,
    GetGuestCount,
    GetProfile,
    MarkProfileSeated,
    SetCurrentGuestProfile,
    SetHostProfile,
)


class GuestMemoryNode(Node):
    def __init__(self):
        super().__init__("guest_memory_node")

        self.declare_parameter("json_path", "~/.ros/rc_task/guest_memory.json")
        self.declare_parameter("appearance_topic", "/appearance/current_estimate")
        self.declare_parameter("enable_json_persistence", True)
        self.declare_parameter("appearance_json_write_period_sec", 1.0)
        self.declare_parameter("default_host_name", "Host")
        self.declare_parameter("default_host_drink", "tea")
        self.declare_parameter("default_host_hometown", "living room")

        self.json_path = Path(str(self.get_parameter("json_path").value)).expanduser()
        self.enable_json_persistence = bool(self.get_parameter("enable_json_persistence").value)
        self.appearance_json_write_period_sec = float(
            self.get_parameter("appearance_json_write_period_sec").value
        )
        self.last_appearance_json_write_time = 0.0

        self.host_profile = self._build_default_host_profile()
        self.current_guest = None
        self.newcomer_profiles = {}
        self.seated_profiles = {}
        self.latest_appearance = self._build_unknown_appearance()

        self.create_subscription(
            GuestAppearance,
            str(self.get_parameter("appearance_topic").value),
            self.appearance_cb,
            10,
        )

        self.create_service(SetHostProfile, "/guest_memory/set_host_profile", self.set_host_cb)
        self.create_service(
            SetCurrentGuestProfile,
            "/guest_memory/set_current_guest_profile",
            self.set_current_guest_cb,
        )
        self.create_service(
            CommitCurrentGuest,
            "/guest_memory/commit_current_guest",
            self.commit_current_guest_cb,
        )
        self.create_service(
            ClearCurrentGuest,
            "/guest_memory/clear_current_guest",
            self.clear_current_guest_cb,
        )
        self.create_service(
            MarkProfileSeated,
            "/guest_memory/mark_profile_seated",
            self.mark_profile_seated_cb,
        )
        self.create_service(GetProfile, "/guest_memory/get_profile", self.get_profile_cb)
        self.create_service(GetGuestCount, "/guest_memory/get_guest_count", self.get_guest_count_cb)

        self.write_json_memory("startup")
        self.get_logger().info("Guest memory prototype is ready.")

    def _build_unknown_appearance(self):
        stamp = self.get_clock().now().to_msg()

        appearance = GuestAppearance()
        appearance.glasses = GuestAppearance.UNKNOWN
        appearance.hat = GuestAppearance.UNKNOWN
        appearance.face_mask = GuestAppearance.UNKNOWN
        appearance.shirt_color = "unknown"
        appearance.glasses_confidence = 0.0
        appearance.hat_confidence = 0.0
        appearance.face_mask_confidence = 0.0
        appearance.shirt_color_confidence = 0.0
        appearance.stamp = stamp
        return appearance

    def _build_default_host_profile(self):
        stamp = self.get_clock().now().to_msg()

        profile = PersonProfile()
        profile.profile_id = "host"
        profile.role = "host"
        profile.name = str(self.get_parameter("default_host_name").value)
        profile.age = 0
        profile.gender = "unknown"
        profile.favorite_drink = str(self.get_parameter("default_host_drink").value)
        profile.hometown = str(self.get_parameter("default_host_hometown").value)
        profile.appearance = self._build_unknown_appearance()
        profile.is_confirmed = bool(profile.name and profile.favorite_drink)
        profile.confidence_score = 1.0 if profile.is_confirmed else 0.0
        profile.stamp = stamp
        profile.appearance.stamp = stamp
        return profile

    def _appearance_has_signal(self, appearance):
        return bool(
            appearance.shirt_color
            or appearance.glasses != appearance.UNKNOWN
            or appearance.hat != appearance.UNKNOWN
            or appearance.face_mask != appearance.UNKNOWN
        )

    def _merge_appearance(self, existing_appearance, incoming_appearance):
        merged = copy.deepcopy(existing_appearance)

        if incoming_appearance.glasses != incoming_appearance.UNKNOWN and (
            merged.glasses == merged.UNKNOWN
            or incoming_appearance.glasses_confidence >= merged.glasses_confidence
        ):
            merged.glasses = incoming_appearance.glasses
            merged.glasses_confidence = incoming_appearance.glasses_confidence

        if incoming_appearance.hat != incoming_appearance.UNKNOWN and (
            merged.hat == merged.UNKNOWN
            or incoming_appearance.hat_confidence >= merged.hat_confidence
        ):
            merged.hat = incoming_appearance.hat
            merged.hat_confidence = incoming_appearance.hat_confidence

        if incoming_appearance.face_mask != incoming_appearance.UNKNOWN and (
            merged.face_mask == merged.UNKNOWN
            or incoming_appearance.face_mask_confidence >= merged.face_mask_confidence
        ):
            merged.face_mask = incoming_appearance.face_mask
            merged.face_mask_confidence = incoming_appearance.face_mask_confidence

        if incoming_appearance.shirt_color and (
            not merged.shirt_color
            or incoming_appearance.shirt_color_confidence >= merged.shirt_color_confidence
        ):
            merged.shirt_color = incoming_appearance.shirt_color
            merged.shirt_color_confidence = incoming_appearance.shirt_color_confidence

        if self._appearance_has_signal(incoming_appearance):
            merged.stamp = incoming_appearance.stamp

        return merged

    def _merge_profile(self, existing_profile, incoming_profile, profile_id, role):
        merged = copy.deepcopy(existing_profile) if existing_profile is not None else PersonProfile()

        merged.profile_id = incoming_profile.profile_id or profile_id
        merged.role = incoming_profile.role or role

        if incoming_profile.name:
            merged.name = incoming_profile.name
        if incoming_profile.age > 0:
            merged.age = incoming_profile.age
        if incoming_profile.gender:
            merged.gender = incoming_profile.gender
        if incoming_profile.favorite_drink:
            merged.favorite_drink = incoming_profile.favorite_drink
        if incoming_profile.hometown:
            merged.hometown = incoming_profile.hometown

        merged.appearance = self._merge_appearance(merged.appearance, incoming_profile.appearance)
        merged.confidence_score = max(merged.confidence_score, incoming_profile.confidence_score)
        merged.is_confirmed = bool(merged.name and merged.favorite_drink)
        merged.stamp = self.get_clock().now().to_msg()

        if self._appearance_has_signal(merged.appearance):
            merged.appearance.stamp = merged.stamp

        return merged

    def _normalized_guest_profile(self, profile, profile_id):
        normalized = copy.deepcopy(profile)
        normalized.profile_id = profile_id
        normalized.role = "guest"
        normalized.is_confirmed = bool(normalized.name and normalized.favorite_drink)
        normalized.stamp = self.get_clock().now().to_msg()
        normalized.appearance.stamp = normalized.stamp
        return normalized

    def _current_guest_profile_id(self):
        if self.current_guest is None:
            return ""
        return self.current_guest.profile_id or "current_guest"

    def _all_guest_ids(self):
        guest_ids = list(self.newcomer_profiles.keys())
        for profile_id in self.seated_profiles.keys():
            if profile_id not in guest_ids:
                guest_ids.append(profile_id)
        return guest_ids

    def appearance_cb(self, msg):
        self.latest_appearance = copy.deepcopy(msg)
        if not self.latest_appearance.shirt_color:
            self.latest_appearance.shirt_color = "unknown"

        now = time.monotonic()
        if now - self.last_appearance_json_write_time >= self.appearance_json_write_period_sec:
            self.last_appearance_json_write_time = now
            self.write_json_memory("appearance")

    def _enum_to_text(self, value):
        if value == GuestAppearance.YES:
            return "yes"
        if value == GuestAppearance.NO:
            return "no"
        return "unknown"

    def _time_to_dict(self, stamp):
        return {
            "sec": int(stamp.sec),
            "nanosec": int(stamp.nanosec),
        }

    def _appearance_to_dict(self, appearance):
        return {
            "glasses": self._enum_to_text(appearance.glasses),
            "glasses_confidence": float(appearance.glasses_confidence),
            "hat": self._enum_to_text(appearance.hat),
            "hat_confidence": float(appearance.hat_confidence),
            "face_mask": self._enum_to_text(appearance.face_mask),
            "face_mask_confidence": float(appearance.face_mask_confidence),
            "shirt_color": appearance.shirt_color or "unknown",
            "shirt_color_confidence": float(appearance.shirt_color_confidence),
            "stamp": self._time_to_dict(appearance.stamp),
        }

    def _profile_to_dict(self, profile):
        if profile is None:
            return None
        return {
            "profile_id": profile.profile_id,
            "role": profile.role,
            "name": profile.name,
            "age": int(profile.age),
            "gender": profile.gender,
            "favorite_drink": profile.favorite_drink,
            "hometown": profile.hometown,
            "appearance": self._appearance_to_dict(profile.appearance),
            "is_confirmed": bool(profile.is_confirmed),
            "confidence_score": float(profile.confidence_score),
            "stamp": self._time_to_dict(profile.stamp),
        }

    def _json_payload(self):
        return {
            "schema_version": 2,
            "updated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "host": self._profile_to_dict(self.host_profile),
            "current_guest": self._profile_to_dict(self.current_guest),
            "newcomer_profiles": {
                profile_id: self._profile_to_dict(profile)
                for profile_id, profile in self.newcomer_profiles.items()
            },
            "seated_profiles": {
                profile_id: self._profile_to_dict(profile)
                for profile_id, profile in self.seated_profiles.items()
            },
            "all_guest_ids": self._all_guest_ids(),
            "latest_appearance": self._appearance_to_dict(self.latest_appearance),
        }

    def write_json_memory(self, reason):
        if not self.enable_json_persistence:
            return

        try:
            self.json_path.parent.mkdir(parents=True, exist_ok=True)
            temp_path = self.json_path.with_suffix(self.json_path.suffix + ".tmp")
            with temp_path.open("w", encoding="utf-8") as file_handle:
                json.dump(self._json_payload(), file_handle, indent=2, sort_keys=True)
                file_handle.write("\n")
            os.replace(temp_path, self.json_path)
            self.get_logger().debug(f"Wrote guest memory JSON for {reason}: {self.json_path}")
        except Exception as exc:
            self.get_logger().error(f"Failed to write guest memory JSON: {exc}")

    def set_host_cb(self, request, response):
        self.host_profile = self._merge_profile(self.host_profile, request.profile, "host", "host")
        response.success = True
        response.message = "Host profile stored."
        self.write_json_memory("set_host")
        return response

    def set_current_guest_cb(self, request, response):
        merged_profile = self._merge_profile(
            self.current_guest,
            request.profile,
            "current_guest",
            "guest",
        )
        current_profile_id = merged_profile.profile_id or "current_guest"
        self.current_guest = self._normalized_guest_profile(merged_profile, current_profile_id)
        response.success = True
        response.message = (
            f"Current guest stored for {self.current_guest.name or 'unknown guest'} "
            f"as {self.current_guest.profile_id or 'current_guest'}."
        )
        self.write_json_memory("set_current_guest")
        return response

    def commit_current_guest_cb(self, request, response):
        if self.current_guest is None:
            response.success = False
            response.message = "No current guest is available to commit."
            response.confirmed_guest_count = len(self._all_guest_ids())
            response.committed_guest_ids = self._all_guest_ids()
            return response

        committed_profile = self._normalized_guest_profile(self.current_guest, request.new_profile_id)
        self.newcomer_profiles[request.new_profile_id] = committed_profile
        self.current_guest = None

        response.success = True
        response.message = f"Current guest committed into newcomer as {request.new_profile_id}."
        response.confirmed_guest_count = len(self._all_guest_ids())
        response.committed_guest_ids = self._all_guest_ids()
        self.write_json_memory("commit_current_guest")
        return response

    def clear_current_guest_cb(self, request, response):
        del request
        self.current_guest = None
        response.success = True
        response.message = "Current guest cleared."
        self.write_json_memory("clear_current_guest")
        return response

    def mark_profile_seated_cb(self, request, response):
        if request.profile_id not in self.newcomer_profiles:
            response.success = False
            response.message = f"Guest '{request.profile_id}' is not in newcomer memory."
            return response

        seated_profile = self.newcomer_profiles.pop(request.profile_id)
        seated_profile.stamp = self.get_clock().now().to_msg()
        seated_profile.appearance.stamp = seated_profile.stamp
        self.seated_profiles[request.profile_id] = seated_profile

        response.success = True
        response.message = f"Guest '{request.profile_id}' moved from newcomer to seated."
        self.write_json_memory("mark_profile_seated")
        return response

    def get_profile_cb(self, request, response):
        if request.profile_id == "current_guest" and self.current_guest is not None:
            response.success = True
            response.message = "Current guest profile found."
            response.profile = copy.deepcopy(self.current_guest)
            return response

        if (
            self.current_guest is not None
            and request.profile_id
            and request.profile_id == self._current_guest_profile_id()
        ):
            response.success = True
            response.message = "Current guest profile found by active guest id."
            response.profile = copy.deepcopy(self.current_guest)
            return response

        if request.profile_id == "host" and self.host_profile is not None:
            response.success = True
            response.message = "Host profile found."
            response.profile = copy.deepcopy(self.host_profile)
            return response

        if request.profile_id in self.newcomer_profiles:
            response.success = True
            response.message = "Newcomer profile found."
            response.profile = copy.deepcopy(self.newcomer_profiles[request.profile_id])
            return response

        if request.profile_id in self.seated_profiles:
            response.success = True
            response.message = "Seated profile found."
            response.profile = copy.deepcopy(self.seated_profiles[request.profile_id])
            return response

        response.success = False
        response.message = f"Profile '{request.profile_id}' was not found."
        response.profile = PersonProfile()
        return response

    def get_guest_count_cb(self, request, response):
        del request
        guest_ids = self._all_guest_ids()
        response.success = True
        response.message = "Guest count computed."
        response.confirmed_guest_count = len(guest_ids)
        response.committed_guest_ids = guest_ids
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GuestMemoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
