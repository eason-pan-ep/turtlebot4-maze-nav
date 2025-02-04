from setuptools import find_packages, setup

package_name = '5335-final-project-jel'

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
            'final-project-first-node = 5335-final-project-jel.final-project-first-node:main'
        ],
    },
)
