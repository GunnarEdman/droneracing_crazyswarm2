from setuptools import find_packages, setup

package_name = 'cs2_test'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'simple_thrust = cs2_test.simple_thrust:main',
        'minimal_test = cs2_test.minimal_test:main',
        'motors_spin = cs2_test.motors_spin:main',
        'pid_pos_controller = cs2_test.pid_pos_controller:main',
        'trajectory_publisher = cs2_test.trajectory_publisher:main',
        ],
    },
)
