cmake_minimum_required(VERSION 3.0.2)
project(n_desc)

# Description only: catkin just installs the files, nothing is compiled.
find_package(catkin REQUIRED)

catkin_package()

install(FILES alpha.urdf.xacro LICENSE
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY xacro meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
