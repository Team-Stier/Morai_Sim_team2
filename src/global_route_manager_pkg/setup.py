#!/usr/bin/env python3
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(**generate_distutils_setup(packages=['global_route_manager_pkg'], package_dir={'': 'src'}))
