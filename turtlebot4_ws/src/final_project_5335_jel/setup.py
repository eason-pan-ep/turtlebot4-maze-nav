from setuptools import find_packages, setup

package_name = 'final_project_5335_jel'  # Changed to match directory name

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
    maintainer='eason',
    maintainer_email='eason.pan@proton.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'final_project_first_node = final_project_5335_jel.final_project_first_node:main'  # Use underscores
        ],
    },
)