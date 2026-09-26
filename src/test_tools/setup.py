from setuptools import find_packages, setup

package_name = 'test_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/test_tools.yaml']),
        ('share/' + package_name + '/launch', ['launch/test_tools.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yanjiun68',
    maintainer_email='pehyanjiun@gmail.com',
    description='Configurable ROS 2 publisher and subscriber nodes for troubleshooting topics.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'test_publisher = test_tools.publisher_node:main',
            'test_subscriber = test_tools.subscriber_node:main',
        ],
    },
)
