from glob import glob

from setuptools import setup

package_name = 'dio13_monitor_example'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAI',
    maintainer_email='user@example.com',
    description='Example ROS 2 Python consumer for an Arduino Nano D2 active-low input.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nano_d2_monitor_node = dio13_monitor_example.dio13_monitor_node:main',
            # Compatibility executable for existing launch scripts.
            'dio13_monitor_node = dio13_monitor_example.dio13_monitor_node:main',
        ],
    },
)
