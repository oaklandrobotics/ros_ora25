from setuptools import find_packages, setup

package_name = 'ora_interface'

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
    maintainer='Zach Lain',
    maintainer_email='ztlain@oakland.edu',
    description='Interface between the software and hardware teams. Built around the MCP2515 CAN chip we are using to communicate',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
