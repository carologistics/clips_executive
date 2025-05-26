from setuptools import find_packages, setup

package_name = 'cxrl_gym'

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
    maintainer='frankpier',
    maintainer_email='frank.pier@rwth-aachen.de',
    description='TODO: Package description',
    license='GPLv2+ license',
    entry_points={
        'console_scripts': [
        ],
    },
)
