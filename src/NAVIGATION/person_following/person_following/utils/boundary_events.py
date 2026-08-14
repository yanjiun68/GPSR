#!/usr/bin/env python3
# -*- coding: utf-8 -*-

EVENT_ZONE_TRIGGERED = 'zone_triggered'

ROLE_EXIT = 'exit'
ROLE_ENTER = 'enter'
VALID_ROLES = {ROLE_EXIT, ROLE_ENTER}

REGION_INSIDE = 'inside'
REGION_OUTSIDE = 'outside'
VALID_REGIONS = {REGION_INSIDE, REGION_OUTSIDE}


def normalize_role(value, default: str = ROLE_EXIT) -> str:
    role = str(value or '').strip().lower()
    if role in VALID_ROLES:
        return role
    return default


def is_valid_role(value) -> bool:
    return str(value or '').strip().lower() in VALID_ROLES


def normalize_region(value, default: str = REGION_INSIDE) -> str:
    region = str(value or '').strip().lower()
    if region in VALID_REGIONS:
        return region
    return default


def is_valid_region(value) -> bool:
    return str(value or '').strip().lower() in VALID_REGIONS
