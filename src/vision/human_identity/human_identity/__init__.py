"""Persistent visual human identity components."""

from .database import IdentityDatabase
from .feature_types import ATTRIBUTE_FALSE, ATTRIBUTE_TRUE, ATTRIBUTE_UNKNOWN
from .matcher import IdentityMatcher
from .registration_manager import RegistrationManager

__all__ = [
    "ATTRIBUTE_FALSE",
    "ATTRIBUTE_TRUE",
    "ATTRIBUTE_UNKNOWN",
    "IdentityDatabase",
    "IdentityMatcher",
    "RegistrationManager",
]
