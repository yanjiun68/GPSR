"""Synthetic JointState extraction checks that require no physical hardware."""

from sensor_msgs.msg import JointState


def extract_named_feedback(message: JointState):
    """Mirror the node's name-based motor and finger extraction contract."""
    motor_index = message.name.index("gripper")
    finger_index = message.name.index("left_finger")
    return (
        message.position[motor_index],
        message.velocity[motor_index],
        message.effort[motor_index],
        message.position[finger_index],
    )


def test_synthetic_feedback_ignores_virtual_finger_effort() -> None:
    """Use gripper motor velocity and effort even when joint ordering changes."""
    message = JointState()
    message.name = ["left_finger", "waist", "gripper", "right_finger"]
    message.position = [0.021, 0.0, 1.25, -0.021]
    message.velocity = [0.0, 0.0, -0.15, 0.0]
    message.effort = [0.0, 0.0, 7.5, 0.0]
    assert extract_named_feedback(message) == (1.25, -0.15, 7.5, 0.021)
