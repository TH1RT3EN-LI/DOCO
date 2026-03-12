from setuptools import find_packages
from setuptools import setup

setup(
    name='uav_visual_landing',
    version='0.0.0',
    packages=find_packages(
        include=('uav_visual_landing', 'uav_visual_landing.*')),
)
