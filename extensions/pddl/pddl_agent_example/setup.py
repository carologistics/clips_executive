from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pddl_agent_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'clips'), glob('clips/*')),
        (os.path.join('share', package_name, 'pddl'), glob('pddl/*')),
        (os.path.join('share', package_name, 'params'), glob('params/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Swoboda',
    maintainer_email='daniel.swoboda@ml.rwth-aachen.de',
    description='Simple example package for how to set up a CX agent with PDDL Manager interface.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pddl_agent_example = pddl_agent_example.pddl_agent_example:main'
        ],
    },
)
