from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(**generate_distutils_setup(
    packages=['localization_pkg'],
    package_dir={'': 'src'},
))
