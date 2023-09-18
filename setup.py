from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'wili_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shinagawa Kazemaru',
    maintainer_email='marukazemaru0@gmail.com',
    description='socket server for WiLI(Where is Lost Item)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'socket_bridge = wili_bridge.socket_bridge:main'
        ],
    },
)
