from setuptools import setup
from setuptools import find_packages # https://zhuanlan.zhihu.com/p/460233022 4.1 包内必须要 __init__.py 文件

package_name = 'car_controller_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            # f"auto_control_lanedet = {package_name}.auto_control_lanedet:main",
            f"manual_control = {package_name}.manual_control:main",
        ],
    },
)
