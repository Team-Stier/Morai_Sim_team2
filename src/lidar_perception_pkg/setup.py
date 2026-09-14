from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=['lidar_perception_pkg'], package_dir={'': 'src'}))
