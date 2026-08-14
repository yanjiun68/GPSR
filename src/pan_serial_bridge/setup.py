from setuptools import find_packages, setup

package_name = 'pan_serial_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='yanjiun68',
    maintainer_email='pehyanjiun@gmail.com',
    description='ROS 2 serial bridge for camera pan commands to an Arduino controller.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_pan = pan_serial_bridge.camera_pan:main',
        ],
    },
)
