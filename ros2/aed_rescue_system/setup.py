import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aed_rescue_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 声明 launch 文件目录
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='student@university.edu',
    description='Intelligent AED Rescue Vehicle System for ELF2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 这里是关键！告诉 colcon 生成哪些可执行文件
            'aed_fall_detection_node = aed_rescue_system.aed_fall_detection_node:main',
            'aed_dispatcher_node = aed_rescue_system.aed_dispatcher_node:main',
            'aed_initializer_node = aed_rescue_system.aed_initializer_node:main'
        ],
    },
)