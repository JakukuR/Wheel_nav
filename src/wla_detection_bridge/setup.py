from setuptools import setup
setup(name='wla_detection_bridge', version='0.1.0', packages=['wla_detection_bridge'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/wla_detection_bridge']),
                  ('share/wla_detection_bridge', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='WLA', maintainer_email='maintainers@example.invalid',
      description='Source-timed shared Agent detection contract and filters', license='Apache-2.0')
