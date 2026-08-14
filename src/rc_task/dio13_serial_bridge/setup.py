from setuptools import setup

package_name = 'dio13_serial_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dio13_serial_bridge.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS 2 Python serial bridge for Arduino DIO13 state reporting.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dio13_serial_publisher = dio13_serial_bridge.dio13_serial_publisher:main',
        ],
    },
)
