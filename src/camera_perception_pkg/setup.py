from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    packages=['camera_perception_pkg', 'camera_perception_pkg.pidnet_model'],
    package_dir={'': 'src'},
    package_data={'camera_perception_pkg.pidnet_model': ['LICENSE']},
))
