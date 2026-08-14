#!/usr/bin/env python3

import argparse
import glob
import json
import os
import re
import sys
import time
from typing import Optional

import serial


BASE_PATTERNS = [
    "/dev/serial/by-id/usb-1a86*",
    "/dev/ttyUSB*",
    "/dev/ttyACM*",
]


def unique_existing_ports(patterns: list[str]) -> list[str]:
    out: list[str] = []
    seen: set[str] = set()
    for pattern in patterns:
        for path in sorted(glob.glob(pattern)):
            if not os.path.exists(path):
                continue
            real = os.path.realpath(path)
            if real in seen:
                continue
            seen.add(real)
            out.append(path)
    return out


def parse_base_ticks(line: str) -> bool:
    parts = re.findall(r"-?\d+", line)
    return len(parts) >= 4


def probe_base(port: str, baud: int, settle_s: float) -> bool:
    try:
        with serial.Serial(port, baud, timeout=0.2, write_timeout=0.2, exclusive=True) as ser:
            time.sleep(settle_s)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(b"e\r")
            deadline = time.monotonic() + 0.6
            while time.monotonic() < deadline:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line or line.upper() == "OK" or line.startswith(("t", "m")):
                    continue
                if parse_base_ticks(line):
                    return True
    except Exception:
        return False
    return False


def probe_lift(port: str, baud: int, settle_s: float) -> bool:
    try:
        with serial.Serial(port, baud, timeout=0.2, write_timeout=0.2, exclusive=True) as ser:
            time.sleep(settle_s)
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                upper = line.upper()
                if not upper:
                    continue
                if upper.startswith("POS") or "HOMED" in upper or "HOMING" in upper:
                    return True
    except Exception:
        return False
    return False


def probe_dio13(port: str, baud: int, settle_s: float) -> bool:
    try:
        with serial.Serial(port, baud, timeout=0.2, write_timeout=0.2, exclusive=True) as ser:
            time.sleep(settle_s)
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                line = ser.readline().decode("utf-8", errors="ignore").strip().upper()
                if line in {"DIO13=HIGH", "DIO13=LOW"}:
                    return True
    except Exception:
        return False
    return False


def _assign_single_remaining(
    ports: list[str], assigned: set[str], role_name: str, debug: list[str]
) -> Optional[str]:
    remaining = [port for port in ports if port not in assigned]
    if len(remaining) == 1:
        chosen = remaining[0]
        debug.append(f"{role_name}_fallback {chosen}")
        return chosen
    if remaining:
        debug.append(f"{role_name}_fallback_ambiguous {remaining}")
    return None


def resolve_roles(
    ports: list[str], baud: int
) -> tuple[Optional[str], Optional[str], Optional[str], list[str]]:
    debug: list[str] = []
    base_port: Optional[str] = None
    lift_port: Optional[str] = None
    dio13_port: Optional[str] = None

    for port in ports:
        is_base = probe_base(port, baud, settle_s=1.0)
        debug.append(f"base_probe {port} -> {is_base}")
        if is_base and base_port is None:
            base_port = port

    for port in ports:
        if port == base_port:
            continue
        is_lift = probe_lift(port, baud, settle_s=1.0)
        debug.append(f"lift_probe {port} -> {is_lift}")
        if is_lift and lift_port is None:
            lift_port = port

    assigned = {port for port in (base_port, lift_port) if port is not None}

    for port in ports:
        if port in assigned:
            continue
        is_dio13 = probe_dio13(port, baud, settle_s=1.0)
        debug.append(f"dio13_probe {port} -> {is_dio13}")
        if is_dio13 and dio13_port is None:
            dio13_port = port

    assigned = {port for port in (base_port, lift_port, dio13_port) if port is not None}

    if lift_port is not None and base_port is None:
        base_port = _assign_single_remaining(ports, assigned, "base", debug)
        assigned = {port for port in (base_port, lift_port, dio13_port) if port is not None}

    if base_port is not None and lift_port is None:
        lift_port = _assign_single_remaining(ports, assigned, "lift", debug)
        assigned = {port for port in (base_port, lift_port, dio13_port) if port is not None}

    if dio13_port is None:
        dio13_port = _assign_single_remaining(ports, assigned, "dio13", debug)

    return base_port, lift_port, dio13_port, debug


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    ports = unique_existing_ports(BASE_PATTERNS)
    base_port, lift_port, dio13_port, debug = resolve_roles(ports, args.baud)
    payload = {
        "ports": ports,
        "base_port": base_port,
        "lift_port": lift_port,
        "dio13_port": dio13_port,
        "debug": debug,
    }

    if args.json:
        print(json.dumps(payload))
    else:
        print(f"ports={ports}")
        print(f"base_port={base_port}")
        print(f"lift_port={lift_port}")
        print(f"dio13_port={dio13_port}")
        for line in debug:
            print(line)

    if base_port is None or lift_port is None or dio13_port is None:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
