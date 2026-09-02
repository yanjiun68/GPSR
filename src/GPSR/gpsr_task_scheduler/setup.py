"""Section: Install the deterministic GPSR scheduler as an ament Python package."""

# Section: Packaging dependencies and setuptools entrypoint.
from glob import glob
from setuptools import find_packages, setup


# Section: Package name used by ament_python.
package_name = "gpsr_task_scheduler"


# Section: Install scheduler modules, config, launch files, and console entrypoints.
setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yanjiun68",
    maintainer_email="yanjiun68@example.com",
    description="Deterministic LLM-absent GPSR task scheduler.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gpsr_scheduler = gpsr_task_scheduler.scheduler_node:main",
            "gpsr_parse_dry_run = gpsr_task_scheduler.parse_dry_run:main",
            "gpsr_validate_sequences = gpsr_task_scheduler.sequence_validator:main",
        ],
    },
)
