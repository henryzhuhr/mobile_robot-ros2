from setuptools import setup

package_name = 'py_srvcli'

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
    maintainer='ubuntu',
    maintainer_email='example_email@gmail.com',
    description='py_srvcli',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':
            [
                'service = py_srvcli.service_member_function:main',
                'client = py_srvcli.client_member_function:main',
            ],
    },
)
