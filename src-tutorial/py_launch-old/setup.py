import glob
from setuptools import setup

package_name = 'py_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),


        # launch 文件相关配置, 无论该功能包的launch目录下有多少个launch文件，launch相关配置只需设置一次即可
        ('share/' + package_name, glob.glob("launch/py/*.launch.py")),
        ('share/' + package_name, glob.glob("launch/xml/*.launch.xml")),
        ('share/' + package_name, glob.glob("launch/yaml/*.launch.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='296506195@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
