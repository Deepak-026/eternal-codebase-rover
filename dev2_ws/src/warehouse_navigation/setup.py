from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'warehouse_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config/maps'), glob('config/maps/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team_39',
    maintainer_email='team_39@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'broadcast = warehouse_navigation.odom_tf_broadcaster:main',
            'odom_to_joint = warehouse_navigation.odom_to_joint_state:main',
            'odom_bridge = warehouse_navigation.odom_raw_to_odom:main',
            'scan_relay = warehouse_navigation.scan_relay:main',
        ],
    },
)
