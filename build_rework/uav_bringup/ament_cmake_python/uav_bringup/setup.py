from setuptools import find_packages
from setuptools import setup

setup(
    name='uav_bringup',
    version='0.1.0',
    packages=find_packages(
        include=('uav_bringup', 'uav_bringup.*')),
)
