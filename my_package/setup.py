from setuptools import find_packages, setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mihir',
    maintainer_email='mvemuri@olin.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
            'wall_following = my_package.wall_following:main',
            'drive_square = my_package.drive_square:main',
            'teleop = my_package.teleop:main',
            'finite_state_controller = my_package.finite_state_controller:main'
            'teleop = my_package.teleop:main',
            'person_following = my_package.person_following:main',
            'obstacle_avoidance = my_package.obstacle_avoidance:main'
        ],
    },
)