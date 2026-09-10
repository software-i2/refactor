cmake_minimum_required(VERSION 3.0.2)
project(n_driver)

set(EXTRA_CATKIN_PROJECTS n_conf)

find_file(BEEX_CMAKE_INCLUDE bx_msgs/include/BEEX_CMAKE.txt HINTS ../../ ../ REQUIRED)
include(${BEEX_CMAKE_INCLUDE})

compile_my_exec(n_driver_node
                src/main.cpp
                src/Node.cpp
                src/Arm.cpp
                src/Packet.cpp
                src/Serial.cpp
                src/Sim.cpp)
target_link_libraries(n_driver_node pthread)

set(INSTALL_SUPPORT_FOLDERS urdf)
install_my_config_and_launch_files()
