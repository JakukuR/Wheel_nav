from setuptools import setup
from glob import glob

name = 'wla_person_follow'
setup(name=name, version='0.2.0', packages=[name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]),
                ('share/'+name, ['package.xml', 'README.md']),
                ('share/'+name+'/config', glob('config/*.yaml')),
                ('share/'+name+'/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='JakukuR', maintainer_email='1330367040@qq.com',
    description='Relative person approach and follow with explicit Agent commands',
    license='MIT', scripts=['scripts/person_follow'])
