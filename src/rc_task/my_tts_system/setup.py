from setuptools import setup
from glob import glob

package_name = 'my_tts_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Guest text renderer and Piper TTS bridge for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'guest_text_renderer = my_tts_system.guest_text_renderer:main',
            'json_guest_text_renderer = my_tts_system.json_guest_text_renderer:main',
            'piper_tts_node = my_tts_system.piper_tts_node:main',
            'test_guest_profile_publisher = my_tts_system.test_guest_profile_publisher:main',
        ],
    },
)
