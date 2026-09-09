from setuptools import setup, find_packages

package_name = 'buildscan_hardware'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/hardware.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BuildScan Team',
    maintainer_email='buildscan@example.com',
    description='Hardware interface nodes for BuildScan Rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_interface_node = buildscan_hardware.motor_interface_node:main',
            'safety_node = buildscan_hardware.safety_node:main',
        ],
    },
)
