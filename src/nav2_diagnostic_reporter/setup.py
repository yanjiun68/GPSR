"""Setuptools configuration for the Nav2 diagnostic reporter."""

# Section: Standard Python package setup imports.
from glob import glob
from setuptools import setup


# Section: ROS 2 ament Python package registration.
package_name = 'nav2_diagnostic_reporter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yanjiun68',
    maintainer_email='yanjiun68@example.com',
    description='Standalone Nav2 NavigateToPose diagnostic proxy.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diagnostic_reporter = nav2_diagnostic_reporter.diagnostic_proxy:main',
            'diagnostic_user_message_tts_bridge = nav2_diagnostic_reporter.diagnostic_user_message_tts_bridge:main',
        ],
    },
)
