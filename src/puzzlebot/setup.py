from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'puzzlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TU_NOMBRE',
    maintainer_email='tu.email@ejemplo.com',
    description='Nodo ROS 2 para navegación hacia waypoints usando /cmd_vel',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'traffic_light_node = puzzlebot.traffic_light_node:main',
        'image_viewer_node = puzzlebot.image_viewer_node:main',
        'ml_yolo = puzzlebot.ml_yolo:main',
        'master_node = puzzlebot.master_node:main',
    ],
    },
)

