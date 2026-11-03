from setuptools import setup, find_packages
from glob import glob

name = 'wla_explore_wander'
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]),
                ('share/'+name, ['package.xml', 'README.md']),
                ('share/'+name+'/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='JakukuR', maintainer_email='1330367040@qq.com',
    description='Wander and curiosity exploration driven through the person-follow MPC legs',
    license='MIT', entry_points={'console_scripts': ['explore_wander = wla_explore_wander.node:main']})
