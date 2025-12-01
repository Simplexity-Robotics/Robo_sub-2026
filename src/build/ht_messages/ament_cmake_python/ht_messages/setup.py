from setuptools import find_packages
from setuptools import setup

setup(
    name='ht_messages',
    version='0.0.0',
    packages=find_packages(
        include=('ht_messages', 'ht_messages.*')),
)
