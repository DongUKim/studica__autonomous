import os
from glob import glob

from setuptools import setup

package_name = 'studica_repeat'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.core'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='studica_national',
    maintainer_email='deeptree00@gmail.com',
    description='Plan B teach-and-repeat navigation (recorder, repeat controller, validation, sim).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teach_node = studica_repeat.teach_node:main',
            'repeat_node = studica_repeat.repeat_node:main',
            'validate_node = studica_repeat.validate_node:main',
            'sim_node = studica_repeat.sim_node:main',
            'send_route = studica_repeat.send_route:main',
        ],
    },
)
