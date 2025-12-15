from setuptools import find_packages
from setuptools import setup

setup(
    name='grid_map_msgs',
    version='2.1.0',
    packages=find_packages(
        include=('grid_map_msgs', 'grid_map_msgs.*')),
)
