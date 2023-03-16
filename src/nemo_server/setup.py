from setuptools import setup

package_name = 'nemo_server'
lib = package_name + '/lib'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, lib],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'localiser = nemo_server.localiser:main'
        ],
    },
)
