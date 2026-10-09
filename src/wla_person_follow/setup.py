from setuptools import setup

name = 'wla_person_follow'
setup(
    name=name,
    version='0.1.0',
    packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md']),
        ('share/' + name + '/config', ['config/follow.yaml']),
        ('share/' + name + '/launch', ['launch/follow.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JakukuR',
    maintainer_email='1330367040@qq.com',
    description='D455 person following with identity lock and candidate velocity',
    license='MIT',
    scripts=['scripts/person_follow'],
)
