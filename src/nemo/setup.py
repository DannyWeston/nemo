import os
import glob

from setuptools import setup

package_name = 'nemo'

lib = os.path.join(package_name, 'lib')

hardware = os.path.join(package_name, 'hardware')
localiser = os.path.join(package_name, 'localiser')
planner = os.path.join(package_name, 'planner')
recorder = os.path.join(package_name, 'recorder')

setup(
    name=package_name,
    version='0.0.1',
    packages=[hardware, localiser, planner, recorder, lib],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml'))
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Weston',
    maintainer_email='danielweston37@gmail.com',
    description='NEMO Drone Firmware',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware = nemo.hardware.hardware:main',
            'planner = nemo.planner.planner:main',
            'recorder = nemo.recorder.recorder:main',
            'localiser = nemo.localiser.localiser:main',
        ],
    },
)

