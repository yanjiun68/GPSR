from glob import glob

from setuptools import setup

package_name = 'dio13_serial_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS 2 Python serial bridge for an Arduino Nano D2 active-low input.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nano_d2_serial_publisher = dio13_serial_bridge.dio13_serial_publisher:main',
            # Compatibility executable for existing launch scripts.
            'dio13_serial_publisher = dio13_serial_bridge.dio13_serial_publisher:main',
        ],
    },
)
