"""Section: Source-tree import support for hardware-free package tests."""

from __future__ import annotations

import sys
from pathlib import Path


# Section: Test the checked-out Python package rather than a previously installed copy.
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
