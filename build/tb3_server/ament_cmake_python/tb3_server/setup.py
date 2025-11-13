from setuptools import find_packages
from setuptools import setup

setup(
    name='tb3_server',
    version='0.1.0',
    packages=find_packages(
        include=('tb3_server', 'tb3_server.*')),
)
