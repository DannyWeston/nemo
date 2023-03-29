import os
import glob

from setuptools import setup

package_name = 'nemo'
lib = os.path.join(package_name, 'lib')

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, lib],
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
    description='Server for the Nemo drone',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner = nemo.planner:main',
            'camera = nemo.lib.camera:main'
        ],
    },
)

