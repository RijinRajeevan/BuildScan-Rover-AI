from setuptools import setup, find_packages

package_name = 'buildscan_dashboard'

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
    description='Streamlit ROS2 UI dashboard. Runs on laptop.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = buildscan_dashboard.dashboard_node:main',
        ],
    },
)
