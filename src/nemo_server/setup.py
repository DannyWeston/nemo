from setuptools import setup

package_name = 'nemo_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
            'camera_sub = nemo_server.camera_sub:main',
            'pressure_sub = nemo_server.pressure_sub:main'
        ],
    },
)
