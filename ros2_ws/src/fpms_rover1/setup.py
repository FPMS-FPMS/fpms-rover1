from setuptools import setup

package_name = 'fpms_rover1'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rover1.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FPMS',
    maintainer_email='nwadhawan@gmail.com',
    description='FPMS Rover 1 ROS 2 nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'd500_node = fpms_rover1.d500_node:main',
            'thermal_node = fpms_rover1.thermal_node:main',
            'yahboom_node = fpms_rover1.yahboom_node:main',
        ],
    },
)
