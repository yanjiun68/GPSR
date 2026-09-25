#!/usr/bin/env python3

# Import copy so the node can return independent profile snapshots to callers.
import copy

# Import the core ROS 2 Python client library.
import rclpy
# Import the Node base class used to build ROS 2 nodes.
from rclpy.node import Node

# Import the profile message used throughout the receptionist task.
from robot_ai_interfaces.msg import PersonProfile
# Import the services that this memory node must expose.
from robot_ai_interfaces.srv import (
    ClearCurrentGuest,
    CommitCurrentGuest,
    GetGuestCount,
    GetProfile,
    MarkProfileSeated,
    SetCurrentGuestProfile,
    SetHostProfile,
)


# Define one ROS 2 node that owns all guest-memory categories for the prototype.
class GuestMemoryNode(Node):
    def __init__(self):
        # Give the ROS 2 node a stable name so other components can find its services.
        super().__init__("guest_memory_node")

        # Keep the host profile in its own dedicated slot.
        self.host_profile = None
        # Keep the guest currently being interviewed in a temporary slot.
        self.current_guest = None
        # Keep newly collected guests here until they have been introduced and seated.
        self.newcomer_profiles = {}
        # Keep guests here after they have been introduced and seated.
        self.seated_profiles = {}

        # Expose one service for storing or updating the known host profile.
        self.create_service(SetHostProfile, "/guest_memory/set_host_profile", self.set_host_cb)
        # Expose one service for writing the temporary current-guest buffer.
        self.create_service(
            SetCurrentGuestProfile,
            "/guest_memory/set_current_guest_profile",
            self.set_current_guest_cb,
        )
        # Expose one service for committing the temporary guest into newcomer memory.
        self.create_service(
            CommitCurrentGuest,
            "/guest_memory/commit_current_guest",
            self.commit_current_guest_cb,
        )
        # Expose one service for clearing the temporary guest buffer.
        self.create_service(
            ClearCurrentGuest,
            "/guest_memory/clear_current_guest",
            self.clear_current_guest_cb,
        )
        # Expose one service for moving a guest from newcomer memory into seated memory.
        self.create_service(
            MarkProfileSeated,
            "/guest_memory/mark_profile_seated",
            self.mark_profile_seated_cb,
        )
        # Expose one service for retrieving any known profile by id.
        self.create_service(GetProfile, "/guest_memory/get_profile", self.get_profile_cb)
        # Expose one service for retrieving the total count of committed guests.
        self.create_service(GetGuestCount, "/guest_memory/get_guest_count", self.get_guest_count_cb)

    # Return true when one appearance message carries any meaningful observation.
    def _appearance_has_signal(self, appearance):
        return bool(
            appearance.shirt_color
            or appearance.glasses != appearance.UNKNOWN
            or appearance.hat != appearance.UNKNOWN
            or appearance.face_mask != appearance.UNKNOWN
        )

    # Merge one incoming appearance update into an existing appearance snapshot.
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

    # Merge one partial profile update into an existing profile snapshot.
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

    # Build one combined list of all committed guest ids across newcomer and seated memory.
    def _all_guest_ids(self):
        # Collect newcomer ids first so insertion order matches task order.
        guest_ids = list(self.newcomer_profiles.keys())
        # Append seated ids after newcomer ids while avoiding duplicates.
        for profile_id in self.seated_profiles.keys():
            if profile_id not in guest_ids:
                guest_ids.append(profile_id)
        # Return the final ordered guest id list to the caller.
        return guest_ids

    # Normalize one incoming profile so it matches receptionist memory expectations.
    def _normalized_guest_profile(self, profile, profile_id):
        # Copy the incoming profile so the node never mutates a caller-owned object.
        normalized = copy.deepcopy(profile)
        # Force the stable profile id expected by the receptionist task.
        normalized.profile_id = profile_id
        # Force the role because this memory is only used for guests in this code path.
        normalized.role = "guest"
        # Store whether the collected profile has the minimum required data.
        normalized.is_confirmed = bool(normalized.name and normalized.favorite_drink)
        # Stamp the profile using the node clock so later lookups know when it was written.
        normalized.stamp = self.get_clock().now().to_msg()
        # Keep the appearance stamp aligned with the profile stamp.
        normalized.appearance.stamp = normalized.stamp
        # Return the normalized copy to the caller.
        return normalized

    # Return the best identifier currently associated with the temporary guest slot.
    def _current_guest_profile_id(self):
        if self.current_guest is None:
            return ""
        return self.current_guest.profile_id or "current_guest"

    # Store or update the host profile.
    def set_host_cb(self, request, response):
        # Merge the incoming update so appearance-only and dialogue-only writes can coexist.
        self.host_profile = self._merge_profile(self.host_profile, request.profile, "host", "host")
        # Report success because the host slot is always writable.
        response.success = True
        # Return a short explanation that can appear in logs or debugging output.
        response.message = "Host profile stored."
        # Return the populated response object to ROS 2.
        return response

    # Store the actively interviewed guest in the temporary current-guest slot.
    def set_current_guest_cb(self, request, response):
        # Merge the incoming update before normalizing so partial guest writes can accumulate.
        merged_profile = self._merge_profile(
            self.current_guest,
            request.profile,
            "current_guest",
            "guest",
        )
        current_profile_id = merged_profile.profile_id or "current_guest"
        # Normalize the guest profile before storing it in the temporary slot.
        self.current_guest = self._normalized_guest_profile(merged_profile, current_profile_id)
        # Report success because the current guest buffer is always writable.
        response.success = True
        # Return a short explanation that includes the guest name when available.
        response.message = (
            f"Current guest stored for {self.current_guest.name or 'unknown guest'} "
            f"as {self.current_guest.profile_id or 'current_guest'}."
        )
        # Return the populated response object to ROS 2.
        return response

    # Commit the temporary guest into newcomer memory using the requested stable id.
    def commit_current_guest_cb(self, request, response):
        # Refuse the request if there is no current guest to commit.
        if self.current_guest is None:
            response.success = False
            response.message = "No current guest is available to commit."
            response.confirmed_guest_count = len(self._all_guest_ids())
            response.committed_guest_ids = self._all_guest_ids()
            return response

        # Normalize the temporary guest again so the committed record has the final stable id.
        committed_profile = self._normalized_guest_profile(self.current_guest, request.new_profile_id)
        # Save the committed guest into newcomer memory.
        self.newcomer_profiles[request.new_profile_id] = committed_profile
        # Clear the temporary slot after a successful commit.
        self.current_guest = None

        # Report success back to the caller.
        response.success = True
        # Include the final assigned profile id in the response message.
        response.message = f"Current guest committed into newcomer as {request.new_profile_id}."
        # Report how many total committed guests are now known.
        response.confirmed_guest_count = len(self._all_guest_ids())
        # Return the ordered list of guest ids currently tracked by memory.
        response.committed_guest_ids = self._all_guest_ids()
        # Return the populated response object to ROS 2.
        return response

    # Clear the temporary current-guest slot.
    def clear_current_guest_cb(self, request, response):
        # Silence the unused request variable because this service has an empty request.
        del request
        # Drop the temporary current guest so the next interview starts cleanly.
        self.current_guest = None
        # Report success because clearing the temporary buffer cannot fail.
        response.success = True
        # Return a short explanation for logs and debugging.
        response.message = "Current guest cleared."
        # Return the populated response object to ROS 2.
        return response

    # Move one guest from newcomer memory into seated memory.
    def mark_profile_seated_cb(self, request, response):
        # Refuse the move if the requested guest is not currently in newcomer memory.
        if request.profile_id not in self.newcomer_profiles:
            response.success = False
            response.message = f"Guest '{request.profile_id}' is not in newcomer memory."
            return response

        # Remove the guest from newcomer memory and keep the returned profile object.
        seated_profile = self.newcomer_profiles.pop(request.profile_id)
        # Refresh the stamp so the seated transition time is visible in memory.
        seated_profile.stamp = self.get_clock().now().to_msg()
        # Keep the appearance stamp aligned with the profile stamp.
        seated_profile.appearance.stamp = seated_profile.stamp
        # Save the profile into seated memory under the same stable id.
        self.seated_profiles[request.profile_id] = seated_profile

        # Report success because the transfer completed.
        response.success = True
        # Return a short explanation for logs and debugging.
        response.message = f"Guest '{request.profile_id}' moved from newcomer to seated."
        # Return the populated response object to ROS 2.
        return response

    # Look up one profile by id across all memory categories.
    def get_profile_cb(self, request, response):
        # Return the temporary profile if the caller explicitly asked for the current guest.
        if request.profile_id == "current_guest" and self.current_guest is not None:
            response.success = True
            response.message = "Current guest profile found."
            response.profile = copy.deepcopy(self.current_guest)
            return response

        # Return the temporary profile when the caller asks for its active guest id.
        if (
            self.current_guest is not None
            and request.profile_id
            and request.profile_id == self._current_guest_profile_id()
        ):
            response.success = True
            response.message = "Current guest profile found by active guest id."
            response.profile = copy.deepcopy(self.current_guest)
            return response

        # Return the host profile when the caller asks for "host" and a host exists.
        if request.profile_id == "host" and self.host_profile is not None:
            response.success = True
            response.message = "Host profile found."
            response.profile = copy.deepcopy(self.host_profile)
            return response

        # Return the profile if it lives in newcomer memory.
        if request.profile_id in self.newcomer_profiles:
            response.success = True
            response.message = "Newcomer profile found."
            response.profile = copy.deepcopy(self.newcomer_profiles[request.profile_id])
            return response

        # Return the profile if it lives in seated memory.
        if request.profile_id in self.seated_profiles:
            response.success = True
            response.message = "Seated profile found."
            response.profile = copy.deepcopy(self.seated_profiles[request.profile_id])
            return response

        # Report failure when no profile matches the requested id.
        response.success = False
        # Return a short explanation for logs and debugging.
        response.message = f"Profile '{request.profile_id}' was not found."
        # Return an empty profile object so the response is always well-formed.
        response.profile = PersonProfile()
        # Return the populated response object to ROS 2.
        return response

    # Report how many committed guests are currently known.
    def get_guest_count_cb(self, request, response):
        # Silence the unused request variable because this service has an empty request.
        del request
        # Compute the ordered list of all committed guest ids.
        guest_ids = self._all_guest_ids()
        # Report success because counting the in-memory categories cannot fail.
        response.success = True
        # Return a short explanation for logs and debugging.
        response.message = "Guest count computed."
        # Return the count across newcomer and seated memory.
        response.confirmed_guest_count = len(guest_ids)
        # Return the ordered guest id list to the caller.
        response.committed_guest_ids = guest_ids
        # Return the populated response object to ROS 2.
        return response


# Provide the standard ROS 2 Python entry point.
def main(args=None):
    # Initialize the ROS 2 client library.
    rclpy.init(args=args)
    # Construct the guest memory node.
    node = GuestMemoryNode()
    # Spin the node so it can serve requests until shutdown.
    rclpy.spin(node)
    # Destroy the node explicitly during shutdown.
    node.destroy_node()
    # Shut down the ROS 2 client library.
    rclpy.shutdown()


# Run the entry point when the file is executed as a script.
if __name__ == "__main__":
    main()
