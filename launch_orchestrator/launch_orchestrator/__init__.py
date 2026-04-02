from setuptools import setup
from glob import glob
import os

package_name = 'launch_orchestrator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),

        # diagnostics config
        (os.path.join('share', package_name, 'diagnostics', 'config'),
         ['diagnostics/config/diagnostics_aggregator.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='...',
    maintainer_email='...',
    description='...',
    license='...',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)