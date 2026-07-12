from setuptools import setup
from glob import glob
import os

package_name = 'franka_grasping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'pyzmq'],
    zip_safe=True,
    
    entry_points={
        'console_scripts': [
            'grasp_node = franka_grasping.grasp_node:main',
            # 'inference_server = franka_grasping.inference_server:main',
        ],
    },
)