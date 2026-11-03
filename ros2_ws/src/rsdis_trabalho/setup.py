from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rsdis_trabalho'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diogo-ferreira',
    maintainer_email='ferreira.and.diogo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'uav_camera_node = rsdis_trabalho.detetion_cilinder:main',
            'ugv_obstacle_avoidance_node = rsdis_trabalho.obstacle_avoidance:main'
        ],
    },
)
