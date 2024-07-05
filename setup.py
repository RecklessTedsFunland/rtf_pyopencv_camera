from setuptools import setup
import os
from glob import glob

package_name = 'rtf_pyopencv_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='walchko@users.noreply.github.com',
    description='Python OpenCV camera node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pycamera_node = rtf_pyopencv_camera.pycamera_node:main',
            'grab_node = rtf_pyopencv_camera.grab_node:main'
        ],
    },
)
