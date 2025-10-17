from setuptools import setup, find_packages
import os
from glob import glob

package_name = "ht_core"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        # Ament index resource
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=["setuptools"],
    entry_points={
        # Declare all nodes here
        "console_scripts": [
            "mission_node = ht_core.mission_node:main",
            "depth_node = ht_core.depth_node:main",


        ],
    },
    zip_safe=True,
)
