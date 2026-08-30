"""Section: Install both Carry My Luggage YASMIN executables and their resources."""

from glob import glob

from setuptools import find_packages, setup


# Section: Keep the package name identical across setup and the ament index marker.
PACKAGE_NAME = "carry_my_luggage_yasmin"


# Section: Register two independently launchable state-machine entry points.
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
    description="YASMIN Carry My Luggage state machines.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "phase1_sm = carry_my_luggage_yasmin.phase1_sm:main",
            "follow_demo_sm = carry_my_luggage_yasmin.follow_demo_sm:main",
        ],
    },
)
