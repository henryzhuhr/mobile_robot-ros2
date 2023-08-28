from setuptools import setup
from setuptools import find_packages # https://zhuanlan.zhihu.com/p/460233022 4.1 包内必须要 __init__.py 文件
from glob import glob
import os
package_name = 'sensor_joy_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob("./scripts/*.launch.py")),
       
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zx',
    maintainer_email='zx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sensor_joy = sensor_joy_py.joy_node:main"
        ],
    },
)
