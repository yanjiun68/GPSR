from glob import glob
from setuptools import find_packages, setup

package_name = 'gpsr_sys_ui'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/tools', glob('tools/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Putrabot UPM',
    maintainer_email='putrabotupm@gmail.com',
    author='Putrabot UPM',
    author_email='putrabotupm@gmail.com',
    description='Low-overhead ROS 2 GPSR diagnostic monitor and terminal user interface.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor_core = gpsr_sys_ui.monitor_core:main',
            'monitor_tui = gpsr_sys_ui.monitor_tui:main',
        ],
    },
)
