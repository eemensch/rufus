from setuptools import find_packages, setup
from glob import glob

package_name = 'rufus_sim_game'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/episodes',
         glob('config/episodes/*.yaml')),
        ('share/' + package_name + '/launch',
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'sympy', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='Iman Shames',
    maintainer_email='iman.shames@tuta.io',
    description='Pursuit-evasion episode runner and predicate engine.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'episode_runner = rufus_sim_game.episode_runner:main',
            'world_pose_bridge = '
            'rufus_sim_game.world_pose_bridge:main',
        ],
    },
)
