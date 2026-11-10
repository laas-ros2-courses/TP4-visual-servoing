import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tiago_visual_servoing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cpene',
    maintainer_email='pene.clement@gmail.com',
    description='Asservissement visuel de la tete de TIAGo sur un marqueur ArUco (TP4)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aruco_tracker = tiago_visual_servoing.aruco_tracker:main',
            'head_servo = tiago_visual_servoing.head_servo:main',
        ],
    },
)
