# Copyright by BeeX [2026]
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=["n_live"], package_dir={"": "src"}))
