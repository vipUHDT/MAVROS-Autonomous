from setuptools import find_packages
from setuptools import setup

setup(
    name='mavros',
    version='2.12.0',
    packages=find_packages(
        include=('mavros', 'mavros.*')),
)
