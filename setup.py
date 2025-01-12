from setuptools import setup

package_name = 'mavrospy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/offb_launch.py']),  # Add launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Bernas',
    maintainer_email='andrewrbernas@gmail.com',
    description='ROS node to interact with MAVROS for basic UAV control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offb_node_py = mavrospy.offb_node:main', 
        ],
    },
)

