#!/usr/bin/env python3
# -*- coding: utf-8 -*-

MODE_MAPPED = 'mapped'
MODE_UNMAPPED = 'unmapped'
VALID_MODES = {MODE_MAPPED, MODE_UNMAPPED}


def normalize_mode(value, default=MODE_MAPPED) -> str:
    mode = str(value or '').strip().lower()
    if mode in VALID_MODES:
        return mode
    return default


def is_valid_mode(value) -> bool:
    return str(value or '').strip().lower() in VALID_MODES
