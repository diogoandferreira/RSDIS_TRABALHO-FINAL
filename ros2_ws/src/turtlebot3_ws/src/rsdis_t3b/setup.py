from setuptools import find_packages, setup

package_name = 'rsdis_t3b'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='claudio',
    maintainer_email='1230758@isep.ipp.pt',
    description='Pacote ROS 2 para exploracao e detecao de cilindros',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aspirador = rsdis_t3b.aspirador:main',
            'cilindro = rsdis_t3b.cilindro:main',
        ],
    },
)