"""Install the RX200 gripper guard runtime package."""

from glob import glob

from setuptools import find_packages, setup


# Section: Keep package metadata synchronized with the ament resource marker.
PACKAGE_NAME = "rx200_gripper_guard"


# Section: Install runtime nodes, configuration, launch files, and calibration utility.
setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yanjiun68",
    maintainer_email="yanjiun68@example.com",
    description="RX200 gripper contact guard.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gripper_guard_node = rx200_gripper_guard.gripper_guard_node:main",
            "gripper_calibration_logger = rx200_gripper_guard.gripper_calibration_logger:main",
        ],
    },
)
