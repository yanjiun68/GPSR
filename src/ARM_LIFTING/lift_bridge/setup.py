from setuptools import find_packages, setup

package_name = 'lift_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='kevin@todo.todo',
    description='ROS 2 serial driver and hardware interface for lift mechanism',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lift_serial_driver = lift_bridge.lift_serial_driver:main',
            'joint_state_merger = lift_bridge.joint_state_merger:main',
            'fake_lift_controller = lift_bridge.fake_lift_controller:main',
        ],
    },

)
