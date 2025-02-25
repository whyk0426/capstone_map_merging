import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'capstone_map_merging'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yk',
    maintainer_email='kkyyss426@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'merge_map = capstone_map_merging.merge_map:main',
            'repulsive_force = capstone_map_merging.repulsive_force:main',
        ],
    },
)
