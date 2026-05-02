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
        ],
    },
)
