# agv_nav/setup.py

from setuptools import setup

package_name = 'agv_nav'

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
    maintainer='Your Name',
    maintainer_email='yourname@example.com',
    description='AGV navigation package for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate = agv_nav.navigate:main',
            'patrol = agv_nav.patrol:main',
            'patrol2 = agv_nav.patrol2:main',
            'navigate2 = agv_nav.navigate2:main',
            'patrol3 = agv_nav.patrol3:main',
            'navigate3 = agv_nav.navigate3:main',
            'patrol_and_detect = agv_nav.patrol_and_detect:main',
            'yolo = agv_nav.yolo:main',
            'patrol_and_detect2 = agv_nav.patrol_and_detect2:main',
            'camera_pub = agv_nav.camera_pub:main',
        ],
    },
)
