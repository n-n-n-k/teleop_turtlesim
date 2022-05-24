from setuptools import setup

package_name = 'teleop_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='takasehideki',
    maintainer_email='takasehideki@hal.ipc.i.u-tokyo.ac.jp',
    description='Example of ROS 2 Package for Exercises in Information Physics and Computing 3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_turtlesim_node = teleop_turtlesim.teleop_turtlesim_node:main'
        ],
    },
)
