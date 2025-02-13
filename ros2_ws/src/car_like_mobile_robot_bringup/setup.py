from setuptools import setup
from glob import glob
import os

package_name = 'car_like_mobile_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ament用のリソースファイル
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launchファイル群（launchディレクトリ内の*.launch.py）
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # config ディレクトリのインストール（もし存在する場合）
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 package with both Python and C++ nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],

)
