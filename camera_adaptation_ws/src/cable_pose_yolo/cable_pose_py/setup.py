import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cable_pose_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benat',
    maintainer_email='benat@zubieta.todo',
    description='Deteccion 3D de cables usando YOLOv8-Pose',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = cable_pose_py.detector_node:main'
        ],
    },
)