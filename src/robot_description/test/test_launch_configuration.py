import os
from pathlib import Path
import subprocess
import sys


PACKAGE = Path(__file__).parents[1]
LAUNCH_FILE = PACKAGE / 'launch/base.launch.py'


def test_launch_file_has_valid_python_syntax():
    subprocess.run(
        [sys.executable, '-m', 'py_compile', str(LAUNCH_FILE)],
        check=True, timeout=20)


def test_installed_launch_exposes_required_arguments():
    environment = os.environ.copy()
    environment['ROS_LOG_DIR'] = '/tmp/robot_description_test_logs'
    completed = subprocess.run(
        ['ros2', 'launch', 'robot_description', 'base.launch.py', '--show-args'],
        capture_output=True, text=True, check=True, timeout=20,
        env=environment)
    for argument in ('model', 'use_sim_time', 'use_joint_state_publisher',
                     'use_rviz'):
        assert argument in completed.stdout
