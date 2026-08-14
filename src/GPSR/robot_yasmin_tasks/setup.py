from glob import glob
from setuptools import find_packages, setup


# Section: Package name used by ament_python.
package_name = "robot_yasmin_tasks"


# Section: Install Python modules, launch files, README, and entrypoints.
setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yanjiun68",
    maintainer_email="yanjiun68@example.com",
    description="GPSR YASMIN state machines.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gpsr_executor = robot_yasmin_tasks.gpsr.action_queue_executor_sm:main",
            "gpsr_lifecycle_manager = robot_yasmin_tasks.gpsr.lifecycle_manager:main",
            "gpsr_voice_command = robot_yasmin_tasks.gpsr.voice_command_node:main",
            "count_objects_skill_server = robot_yasmin_tasks.ros_servers.count_objects_skill_server:main",
            "count_people_skill_server = robot_yasmin_tasks.ros_servers.count_people_skill_server:main",
            "find_object_skill_server = robot_yasmin_tasks.ros_servers.find_object_skill_server:main",
            "count_objects_blackboard_tts_node = robot_yasmin_tasks.ros_servers.count_objects_blackboard_tts_node:main",
            "count_people_blackboard_tts_node = robot_yasmin_tasks.ros_servers.count_people_blackboard_tts_node:main",
        ],
    },
)
