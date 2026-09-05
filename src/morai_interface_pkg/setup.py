# -*- coding: utf-8 -*-
# catkin_python_setup()가 사용하는 설치 스크립트. src/의 파이썬 패키지를 노출한다.
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["morai_udp_bridge", "morai_udp_bridge.protocol"],
    package_dir={"": "src"},
)

setup(**setup_args)
