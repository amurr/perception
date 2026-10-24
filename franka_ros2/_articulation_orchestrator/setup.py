from glob import glob

from setuptools import find_packages, setup

package_name = 'articulation_orchestrator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/assets', ['assets/objects_dictionary.yaml']),
        ('share/' + package_name + '/assets/scenes', glob('assets/scenes/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='mursalovartem@gmail.com',
    description='Task logic for the FR3 duo: packets from the table into a tray via moveit_master',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'articulation_orchestrator = articulation_orchestrator.articulation_orchestrator:main',
            'planning_scene_updater = articulation_orchestrator.planning_scene_updater:main',
            'run_placements = articulation_orchestrator.run_placements:main',
            'execution_logger = articulation_orchestrator.execution_logger:main',
        ],
    },
)
