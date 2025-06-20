from setuptools import setup
import os
from glob import glob

package_name = 'spacemouse_joint_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Direct joint control of UR5 using SpaceMouse.',
    entry_points={
        'console_scripts': [
            'joint_controller_node = spacemouse_joint_control.joint_controller_node:main',
        ],
    },
)
