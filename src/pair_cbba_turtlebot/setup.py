from setuptools import find_packages, setup


package_name = 'pair_cbba_turtlebot'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/cbba_webots.launch.py']),
    ('share/' + package_name + '/config', ['config/tasks.yaml']),
]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexandre',
    maintainer_email='thilex12@gmail.com',
    description='Distributed CBBA task allocation for TurtleBot3 robots.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cbba_node = pair_cbba_turtlebot.cbba_node:main',
            'initial_pose_publisher = pair_cbba_turtlebot.initial_pose_publisher:main',
        ],
    },
)
