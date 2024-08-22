from glob import glob
from setuptools import find_packages, setup

package_name = 'motion_controller_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob("./launch/*.launch.py")),
        ('share/' + package_name, glob("./launch/*.launch.xml")),
        ('share/' + package_name, glob("./launch/*.launch.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='26506195@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motion_controller=motion_controller.controller:main",
            "test_motion_controller=motion_controller.test:main",
        ],
    },
)
