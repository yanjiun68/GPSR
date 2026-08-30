"""Stable state, reason, and interface names for the gripper guard."""


# Section: Public action feedback phases.
IDLE = "IDLE"
CLOSING = "CLOSING"
CONTACT_CANDIDATE = "CONTACT_CANDIDATE"
CONTACT_CONFIRMED = "CONTACT_CONFIRMED"
HOLDING = "HOLDING"
STOPPING = "STOPPING"
FAULT = "FAULT"
TIMEOUT = "TIMEOUT"
CANCELLED = "CANCELLED"

# Section: Stable action result reasons.
REASON_CONTACT_CONFIRMED = "contact_confirmed"
REASON_EMPTY_CLOSE = "empty_close"
REASON_CONTACT_TIMEOUT = "contact_timeout"
REASON_FEEDBACK_TIMEOUT = "feedback_timeout"
REASON_HARDWARE_FAULT = "hardware_fault"
REASON_CANCELLED = "cancelled"
REASON_INVALID_GOAL = "invalid_goal"
REASON_MISSING_GRIPPER_JOINT = "missing_gripper_joint"
REASON_COMMAND_FAILURE = "command_failure"

# Section: Default ROS interfaces remain owned by the Interbotix stack.
DEFAULT_JOINT_STATES_TOPIC = "/rx200/sdk_joint_states"
DEFAULT_GRIPPER_COMMAND_TOPIC = "/rx200/commands/joint_single"
DEFAULT_REGISTER_SERVICE = "/rx200/get_motor_registers"
DEFAULT_ACTION_NAME = "/rx200/gripper_guard/close_until_contact"
DEFAULT_DIAGNOSTICS_TOPIC = "/rx200/gripper_guard/diagnostics"
DEFAULT_CONTACT_TOPIC = "/rx200/gripper_guard/contact"
