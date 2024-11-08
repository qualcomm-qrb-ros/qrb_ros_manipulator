# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

from setuptools import setup, find_packages

package_name = 'qrb_ros_manipulator_test'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JiongZhao',
    maintainer_email='quic_jionzhao@quicinc.com',
    description='Manipulator ROS test tools',
    license='BSD-3-Clause-Clear',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_control = src.manipulator_keyboard:main',
        ],
    },
)
