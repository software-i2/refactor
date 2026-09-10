cmake_minimum_required(VERSION 3.0.2)
project(n_ctrl)

# BEEX_CMAKE folds this into its find_package(catkin) and catkin_package(), which
# is what puts n_kine's headers and its library on the line.
set(EXTRA_CATKIN_PROJECTS n_kine n_check n_conf)

find_file(BEEX_CMAKE_INCLUDE bx_msgs/include/BEEX_CMAKE.txt HINTS ../../ ../ REQUIRED)
include(${BEEX_CMAKE_INCLUDE})

# n_driver contributes its joint table, headers only. BEEX_CMAKE exports no
# INCLUDE_DIRS, so its include dir is named here.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../n_driver/include)

compile_my_exec(n_ctrl_node
                src/main.cpp
                src/Node.cpp
                src/Path.cpp
                src/Exec.cpp
                src/Trail.cpp)
target_link_libraries(n_ctrl_node pthread)

# Pure logic only, no ROS. Also runs outside Docker.
compile_my_exec(n_ctrl_check
                test/check.cpp
                src/Path.cpp
                src/Exec.cpp
                src/Trail.cpp)

install_my_config_and_launch_files()
