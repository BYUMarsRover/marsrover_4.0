from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nelson Durrant',
    maintainer_email='snelsondurrant@gmail.com',
    description='Hardware controls for the Mars Rover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_mux = rover_control.drive_mux_node:main',
            'mega_wrapper = rover_control.mega_wrapper_node:main',
            'nano_wrapper = rover_control.nano_wrapper_node:main',
            'mode_switcher = rover_control.mode_switcher_node:main',
        ],
    },
)
