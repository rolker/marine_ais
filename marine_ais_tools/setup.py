from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'marine_ais_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roland Arsenault',
    maintainer_email='roland@ccom.unh.edu',
    description='nodes for decoding and publishing AIS data',
    license='BSD-2-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'ais_parser = marine_ais_tools.ais_parser:main',
          'nmea_replay = marine_ais_tools.nmea_replay:main',
          'ais_contact_tracker = marine_ais_tools.ais_contact_tracker:main',
        ],
    },
)
