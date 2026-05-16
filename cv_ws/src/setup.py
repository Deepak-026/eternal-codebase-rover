from setuptools import find_packages, setup

package_name = 'rack_scanner'

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
    maintainer='team_39',
    maintainer_email='team_39@gmail.com',
    description='QR code and rack scanning package for warehouse robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'process_rack = rack_scanner.image_processor:main',
        ],
    },
      
)
