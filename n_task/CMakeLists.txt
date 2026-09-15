cmake_minimum_required(VERSION 3.0.2)
project(n_task)

set(EXTRA_CATKIN_PROJECTS n_kine n_check n_conf n_check_rrt)

find_file(BEEX_CMAKE_INCLUDE bx_msgs/include/BEEX_CMAKE.txt HINTS ../../ ../ REQUIRED)
include(${BEEX_CMAKE_INCLUDE})

# n_ctrl contributes Bridge.h, its state enum and Path.cpp, so the gate plans
# each leg with the planner n_ctrl will drive it with; n_driver the joint table.
# BEEX_CMAKE exports no INCLUDE_DIRS or LIBRARIES, so both are named here.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../n_ctrl/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/../n_driver/include)
set(N_CTRL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../n_ctrl/src/Path.cpp)

compile_my_exec(n_task_node
                src/main.cpp
                src/Node.cpp
                src/Pick.cpp
                src/PickRrt.cpp
                src/FSM.cpp
                ${N_CTRL_PATH})
target_link_libraries(n_task_node pthread)

# Pure logic only, no ROS. Also runs outside Docker.
compile_my_exec(n_task_check
                test/check.cpp
                src/Pick.cpp
                src/FSM.cpp
                ${N_CTRL_PATH})

install_my_config_and_launch_files()
