from setuptools import setup
from glob import glob
import os

package_name = "astra_er_flow_perception"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "prompts"),
            glob("prompts/*.txt"),
        ),
        (
            os.path.join("share", package_name, "scripts"),
            glob("scripts/*"),
        ),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description=(
        "Astra color-depth registration with UnifoLM-ER-Flow target "
        "localization, ROI depth measurement, and real-time tracking."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "astra_registration_node = "
            "astra_er_flow_perception.registration_node:main",
            "er_flow_tracking_node = "
            "astra_er_flow_perception.er_flow_tracking_node:main",
            "test_mineral_water_bottle = "
            "astra_er_flow_perception.test_mineral_water_bottle:main",
        ],
    },
)
