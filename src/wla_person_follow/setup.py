from setuptools import setup, find_packages
from glob import glob

name = 'wla_person_follow'
setup(name=name, version='0.2.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]),
                ('share/'+name, ['package.xml', 'README.md', 'SIM_DEPLOYMENT.md', 'SIM_CORE_PROVENANCE.json', 'PY_CPP_ARCHITECTURE.md', 'HYBRID_TRANSPORT.md']),
                ('share/'+name+'/config', glob('config/*.yaml')),
                ('share/'+name+'/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='JakukuR', maintainer_email='1330367040@qq.com',
    description='Relative person approach and follow with explicit Agent commands',
    license='MIT', scripts=glob('scripts/person_follow*'))
