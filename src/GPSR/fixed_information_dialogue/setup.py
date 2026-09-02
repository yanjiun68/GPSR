from setuptools import find_packages, setup
from glob import glob
import os

package_name = "fixed_information_dialogue"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="maintainer@example.com",
    description="Deterministic fixed-information dialogue provider for GPSR.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "fixed_information_node = fixed_information_dialogue.fixed_information_node:main",
        "answer_text_tts_bridge = fixed_information_dialogue.answer_text_tts_bridge:main",
    ]},
)
