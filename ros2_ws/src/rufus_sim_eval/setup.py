from setuptools import find_packages, setup
from glob import glob

package_name = 'rufus_sim_eval'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/sweeps',
         glob('config/sweeps/*.yaml')),
    ],
    install_requires=['setuptools', 'pyyaml', 'numpy'],
    zip_safe=True,
    maintainer='Iman Shames',
    maintainer_email='iman.shames@tuta.io',
    description='Pursuit-evasion evaluation harness.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'batch_runner = rufus_sim_eval.batch_runner:main',
        ],
    },
)
