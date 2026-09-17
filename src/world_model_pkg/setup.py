#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup(**generate_distutils_setup(
    packages=["world_model_pkg"],
    package_dir={"": "src"},
))
