from setuptools import setup
import os
from glob import glob

package_name = 'b4m_voice'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('lib/' + package_name, ['scripts/voice_control']),
        ('lib/' + package_name, ['test/b4m_voice_test.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Voice control package for TurtleBot3',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'voice_control = b4m_voice.voice_control:main'
        ],
    },
)
