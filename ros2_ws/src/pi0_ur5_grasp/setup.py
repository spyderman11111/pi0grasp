from setuptools import setup, find_packages

setup(
    name='pi0_ur5_grasp',
    version='0.0.1',
    packages=find_packages(),  
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/pi0_ur5_grasp']),
        ('share/pi0_ur5_grasp', ['package.xml']),
        ('share/pi0_ur5_grasp/launch', ['launch/pi0_grasp.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Minimal π0 policy-based UR5 grasping node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pi0grasp = pi0_ur5_grasp.pi0grasp:main',
            'home_pose = pi0_ur5_grasp.home_pose:main',
        ],
    },
)
