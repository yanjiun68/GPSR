import yaml
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message


def load_message_class(message_type):
    try:
        return get_message(message_type)
    except (AttributeError, ModuleNotFoundError, ValueError) as exc:
        raise ValueError(
            f"Unable to load message type '{message_type}'. "
            "Use the full ROS 2 type name, for example 'std_msgs/msg/String'."
        ) from exc


def make_qos_profile(depth, reliability, durability):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=_parse_reliability(reliability),
        durability=_parse_durability(durability),
    )


def message_from_yaml(message_class, yaml_text):
    data = yaml.safe_load(yaml_text) if yaml_text.strip() else {}
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise ValueError('message_yaml must decode to a mapping of ROS message fields.')

    message = message_class()
    set_message_fields(message, data)
    return message


def _parse_reliability(value):
    normalized = str(value).strip().lower()
    if normalized in ('best_effort', 'besteffort'):
        return ReliabilityPolicy.BEST_EFFORT
    if normalized == 'reliable':
        return ReliabilityPolicy.RELIABLE
    raise ValueError("qos_reliability must be 'reliable' or 'best_effort'.")


def _parse_durability(value):
    normalized = str(value).strip().lower()
    if normalized in ('volatile', 'default'):
        return DurabilityPolicy.VOLATILE
    if normalized in ('transient_local', 'transientlocal'):
        return DurabilityPolicy.TRANSIENT_LOCAL
    raise ValueError("qos_durability must be 'volatile' or 'transient_local'.")
