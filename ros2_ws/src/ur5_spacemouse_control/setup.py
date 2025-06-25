from setuptools import setup

package_name = 'ur5_spacemouse_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='SpaceMouse to MoveIt Servo bridge using libspnav',
    license='MIT',
    entry_points={
        'console_scripts': [
            'spacemouse_servo_bridge = ur5_spacemouse_control.spacemouse_servo_bridge:main'
        ],
    },
)
