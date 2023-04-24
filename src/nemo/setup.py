import os
import glob

from setuptools import setup

package_name = 'nemo'

lib = os.path.join(package_name, 'lib')
data = os.path.join(package_name, "data")

hardware = os.path.join(package_name, 'hardware')
localiser = os.path.join(package_name, 'localiser')
planner = os.path.join(package_name, 'planner')
recorder = os.path.join(package_name, 'recorder')
tracker = os.path.join(package_name, 'tracker')

setup(
    name=package_name,
    version='0.0.1',
    packages=[hardware, localiser, planner, recorder, tracker, lib, data],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    include_package_data = True,

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Weston',
    maintainer_email='danielweston37@gmail.com',
    description='NEMO Drone Firmware',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'hardware = {package_name}.hardware.hardware:main',
            f'planner = {package_name}.planner.planner:main',
            f'recorder = {package_name}.recorder.recorder:main',
            f'localiser = {package_name}.localiser.localiser:main',
            f'camera = {package_name}.hardware.camera:main',
            f'tracker = {package_name}.tracker.tracker:main',
        ],
    },
)

