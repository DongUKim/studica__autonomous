from setuptools import setup

package_name = 'studica_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='studica_national',
    maintainer_email='deeptree00@gmail.com',
    description='Keyboard teleoperation for the Studica 3-wheel omni robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = studica_teleop.keyboard_teleop:main',
        ],
    },
)
