from setuptools import find_packages
from setuptools import setup

setup(
    name='robot_simulation',
    version='0.0.0',
    packages=find_packages(
        include=('robot_simulation', 'robot_simulation.*')),
)
