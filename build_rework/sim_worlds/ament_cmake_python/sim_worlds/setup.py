from setuptools import find_packages
from setuptools import setup

setup(
    name='sim_worlds',
    version='0.1.0',
    packages=find_packages(
        include=('sim_worlds', 'sim_worlds.*')),
)
