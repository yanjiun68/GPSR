#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory


def resolve_package_model_path(package_name: str, path_value: str) -> str:
    path_str = str(path_value or "").strip()
    if not path_str:
        return ""
    if os.path.isabs(path_str):
        return path_str
    try:
        pkg_share = get_package_share_directory(package_name)
    except Exception:
        pkg_share = ""
    if path_str.startswith("models/") and pkg_share:
        return os.path.join(pkg_share, path_str)
    package_models_hint = f"/{package_name}/models/"
    if package_models_hint in path_str and pkg_share:
        return os.path.join(pkg_share, "models", os.path.basename(path_str))
    return path_str
