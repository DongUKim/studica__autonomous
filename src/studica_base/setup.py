import os
from glob import glob

from setuptools import setup

package_name = 'studica_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dalbi',
    maintainer_email='deeptree00@gmail.com',
    description='cmd_vel to Titan omni-wheel base driver with odometry and EMS guard.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_node = studica_base.base_node:main',
        ],
    },
)
