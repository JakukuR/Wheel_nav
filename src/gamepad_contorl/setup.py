from setuptools import setup
import os
from glob import glob

package_name = 'gamepad_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Gamepad teleop controller for differential drive robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gamepad_teleop = gamepad_control.gamepad_teleop:main',
            'keyboard_sim = gamepad_control.keyboard_sim:main',
        ],
    },
)
