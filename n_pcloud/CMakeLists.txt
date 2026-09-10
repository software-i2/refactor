cmake_minimum_required(VERSION 3.0.2)
project(n_pcloud)

# Python only: catkin exports the module and installs the scripts.
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/scene.py scripts/viz.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
