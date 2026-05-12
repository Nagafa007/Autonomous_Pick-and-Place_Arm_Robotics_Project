import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_capstone_brain'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ady',
    maintainer_email='ady@todo.todo',
    description='Capstone brain nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_node = my_capstone_brain.ik_node:main',
            'conveyor_node = my_capstone_brain.conveyor_node:main',
            'vision_node = my_capstone_brain.vision_node:main',
            'control_node = my_capstone_brain.control_node:main',  # Added the new Control Node!
        ],
    },
)