from setuptools import setup
from glob import glob
setup(name='wla_object_follow',version='0.1.0',packages=['wla_object_follow'],
 data_files=[('share/ament_index/resource_index/packages',['resource/wla_object_follow']),
 ('share/wla_object_follow',['package.xml','README.md']),
 ('share/wla_object_follow/config',glob('config/*.yaml')),
 ('share/wla_object_follow/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='JakukuR',maintainer_email='1330367040@qq.com',
 description='Relative object approach and following',license='MIT',
 entry_points={'console_scripts':['object_follow = wla_object_follow.node:main']})
