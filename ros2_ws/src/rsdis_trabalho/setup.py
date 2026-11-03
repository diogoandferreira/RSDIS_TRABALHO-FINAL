from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rsdis_trabalho'


def package_files(directory):
    """Instala uma pasta inteira (com subpastas) em share/<package_name>/<directory>."""
    paths = []
    for root, _, files in os.walk(directory):
        if files:
            paths.append((
                os.path.join('share', package_name, root),
                [os.path.join(root, f) for f in files],
            ))
    return paths


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.sdf'))),
    ]
    + package_files('models')    # model://x3_uav
    + package_files('meshes'),   # model://orchard/orchard_world.dae
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