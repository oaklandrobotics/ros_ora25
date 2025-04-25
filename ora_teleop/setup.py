from setuptools import find_packages, setup

package_name = 'ora_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joystick.launch.py']),
        ('share/' + package_name + '/config', ['config/joystick.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zlain',
    maintainer_email='zacharytlain@gmail.com',
    description='Teleop package with joystick and ODrive reinit',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'reinit_control = ora_teleop.reinit_control:main'
        ],
    },
)
