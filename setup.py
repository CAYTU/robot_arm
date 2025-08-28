from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include URDF files - FIXED PATH
        (os.path.join('share', package_name, 'urdf'),
         glob(os.path.join(package_name, 'urdf', '*'))),
        # Include mesh files - FIXED PATH
        (os.path.join('share', package_name, 'meshes'),
         glob(os.path.join(package_name, 'meshes', '*'))),
        # Include launch files if you have any - FIXED PATH
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join(package_name, 'launch', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akhad',
    maintainer_email='akhad0015@gmail.com',  # Fixed email format
    description='Robot arm package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = robot_arm.publisher:main',
            'publisher_gui = robot_arm.publisher_gui:main',
            'publisher_trossen = robot_arm.publisher_trossen:main'
            # 'isaac_frank_arm = robot_arm.isaac_frank_arm:main',
            # 'gripper = robot_arm.gripper:main',
        ],
    },
)
