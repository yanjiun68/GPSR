from setuptools import find_packages, setup

package_name = 'llm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['../llm/prompt.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miko',
    maintainer_email='mikotankexuan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'client_llm = llm_pkg.client_llm:main',
            'receptionist_advisor_node = llm_pkg.receptionist_advisor_node:main',
            'service_llm = llm_pkg.service_llm:main',
        ],
    },
)
