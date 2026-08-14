"""Section: Source-tree pytest import setup."""

from pathlib import Path
import sys


# Section: Insert package root for direct pytest execution.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
