from setuptools import find_packages, setup

package_name = 'ant_lidar'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lds006.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nils Schrum',
    maintainer_email='urbaninnovation@users.noreply.github.com',
    description='LDS-006 (Ecovacs-Deebot-LiDAR) als sensor_msgs/LaserScan',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = ant_lidar.lidar_node:main',
        ],
    },
)
