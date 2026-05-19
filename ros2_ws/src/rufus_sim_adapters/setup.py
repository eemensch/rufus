from setuptools import find_packages, setup
from glob import glob

package_name = 'rufus_sim_adapters'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Iman Shames',
    maintainer_email='iman.shames@tuta.io',
    description='Pursuit-evasion velocity-command adapters.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rover_adapter = rufus_sim_adapters.rover_adapter:main',
            'quad_adapter = rufus_sim_adapters.quad_adapter:main',
            'fixed_wing_adapter = rufus_sim_adapters.fixed_wing_adapter:main',
        ],
    },
)
