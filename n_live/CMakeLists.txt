cmake_minimum_required(VERSION 3.0.2)
project(n_live)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/live.py scripts/listen.py scripts/planned.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
