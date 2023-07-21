from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,

                                                                                     # 需要与 package.xml 保持一致
    maintainer='ubuntu',
    maintainer_email='296506195@qq.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],

                                                                                     # 添加以下内 console_scripts 支架的 entry_points 领域
    entry_points={
        'console_scripts':
            [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
            ],
    },
)
