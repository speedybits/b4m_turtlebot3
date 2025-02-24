from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'b4m_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name + '/worlds',
         glob(os.path.join('worlds', '*.wbt'))),
    ],
    install_requires=[
        'setuptools',
        'bike4py>=0.1.1',
    ],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='mike.kindig@gmail.com',
    description='ROS2 bridge node for processing sensor data and controlling the Turtlebot3 robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'b4m_bridge = b4m_bridge.b4m_bridge_node:main',
        ],
    },
)
