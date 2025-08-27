from setuptools import setup
import os
from glob import glob

package_name = 'tb3_explorer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='RRT* Global Planner for Nav2 with TB3',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'explorer = tb3_explorer.explorer_node:main',
            'rrt_planner = tb3_explorer.rrt_planner_node:main',
            'semantic_query = tb3_explorer.semantic_query_node:main',
        ]
    },
)
