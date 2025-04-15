from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot4_safety'  # Changed to match directory name

setup(
    name=package_name,
    version='0.2.4',
    packages=[package_name, f'{package_name}.scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.launch.py'))),
        # Config files
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))),
        # Map files
        (os.path.join('share', package_name, 'maps'), 
         glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eason',
    maintainer_email='eason.pan@proton.me',
    description='maze navigation project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_params = turtlebot4_safety.scripts.safety_params:main',
            'obstacle_avoidance_nav = turtlebot4_safety.scripts.obstacle_avoidance_nav:main',
        ],
    },
)