from setuptools import setup
import os
from glob import glob

package_name = 'receptionist_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name,
        f'{package_name}.helpers',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Python support nodes for receptionist task',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'appearance_action_server = receptionist_nodes.appearance_action_server:main',
            'dialogue_action_server = receptionist_nodes.dialogue_action_server:main',
            'guest_profile_memory_bridge = receptionist_nodes.guest_profile_memory_bridge:main',
            'host_profile_memory_bridge = receptionist_nodes.host_profile_memory_bridge:main',
            'guest_memory_node = receptionist_nodes.guest_memory_node:main',
            'introduction_node = receptionist_nodes.introduction_node:main',
            'tts_bridge_node = receptionist_nodes.tts_bridge_node:main',
            'nav_controller_node = receptionist_nodes.nav_controller_node:main',
            'seated_people_node = receptionist_nodes.seated_people_node:main',
            'chair_manager_node = receptionist_nodes.chair_manager_node:main',
            'seat_pointing_node = receptionist_nodes.seat_pointing_node:main',
            'detect_guest_enter = receptionist_nodes.detect_guest_enter:main',
        ],
    },
)
