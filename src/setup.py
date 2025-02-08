from setuptools import find_packages, setup

package_name = 'ce215_pkg'

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
    maintainer='jd22276',
    maintainer_email='jd22276@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'hello_world = ce215_pkg.hw_node:main',
        'publish_velocity = ce215_pkg.pv_node:main',
        'subscribe_odometry = ce215_pkg.so_node:main',
        'subscribe_laser = ce215_pkg.sl_node:main',
        'assignment1 = ce215_pkg.as_node:main',
        'assignment2 = ce215_pkg.as2_node:main',
        'my_robot1 = ce215_pkg.as2_node:main',
        ],
    },
)
