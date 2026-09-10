cmake_minimum_required(VERSION 3.0.2)
project(n_task)

set(EXTRA_CATKIN_PROJECTS n_kine n_check n_conf)

find_file(BEEX_CMAKE_INCLUDE bx_msgs/include/BEEX_CMAKE.txt HINTS ../../ ../ REQUIRED)
include(${BEEX_CMAKE_INCLUDE})

# n_ctrl contributes Bridge.h and its state enum; n_driver the joint table.
# Both are headers only, and BEEX_CMAKE exports no INCLUDE_DIRS.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../n_ctrl/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/../n_driver/include)

compile_my_exec(n_task_node
                src/main.cpp
                src/Node.cpp
                src/Pick.cpp)
target_link_libraries(n_task_node pthread)

# Pure logic only, no ROS. Also runs outside Docker.
compile_my_exec(n_task_check
                test/check.cpp
                src/Pick.cpp)

install_my_config_and_launch_files()
