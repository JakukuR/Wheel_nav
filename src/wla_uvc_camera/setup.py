from setuptools import setup
setup(name='wla_uvc_camera',version='0.1.0',packages=['wla_uvc_camera'],
 data_files=[('share/ament_index/resource_index/packages',['resource/wla_uvc_camera']),
 ('share/wla_uvc_camera',['package.xml','README.md'])],
 install_requires=['setuptools'],entry_points={'console_scripts':[
 'uvc_camera = wla_uvc_camera.node:main',
 'uvc_cameras = wla_uvc_camera.bringup:main',
 'uvc_export_calibration = wla_uvc_camera.export:main',
 'uvc_depth = wla_uvc_camera.pipeline:main']})
