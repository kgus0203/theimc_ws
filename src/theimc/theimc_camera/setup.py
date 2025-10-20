from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slampibot_camera'

setup(
    name=package_name,
    version='7.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Leo Cho',
    author_email='leohycho@gmail.com',
    maintainer='erail',
    maintainer_email='edurobotailab@gmail.com',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS2 Humble Packages for Slampibot',    
    license='Apache License, Version 2.0',       
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_pub = slampibot_camera.slampibot_camera_pub:main',
            'camera_sub = slampibot_camera.slampibot_camera_sub:main',
        ],
    },    
)
