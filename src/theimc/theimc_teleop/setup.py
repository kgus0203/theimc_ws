from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'theimc_teleop'

setup(
    name=package_name,
    version='7.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='jeff',
    author_email='jeff@todo.com',
    maintainer='erail',
    maintainer_email='jeff@todo.com',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS2 Humble Packages',    
    license='Apache License, Version 2.0',
    
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = theimc_teleop.theimc_teleop_keyboard:main',
        ],
    },
)

