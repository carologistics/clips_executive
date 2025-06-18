from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pddl_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'params'), glob('params/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tarik Viehmann',
    maintainer_email='viehmann@kbsg.rwth-aachen.de',
    description='This package allows you to manage PDDL planning environments for use with the ROS2 CX.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pddl_manager = pddl_manager.pddl_manager:main'
        ],
    },
)
