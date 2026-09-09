from setuptools import setup, find_packages

package_name = 'buildscan_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BuildScan Team',
    maintainer_email='buildscan@example.com',
    description='YOLO26n-seg crack detection ROS2 node. Runs on laptop GPU.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crack_detection_node = buildscan_perception.crack_detection_node:main',
        ],
    },
)
