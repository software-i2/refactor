cmake_minimum_required(VERSION 3.0.2)
project(n_check_rrt)

find_package(catkin REQUIRED COMPONENTS n_kine n_conf n_check)

catkin_package(INCLUDE_DIRS include
               LIBRARIES n_check_rrt
               CATKIN_DEPENDS n_kine n_conf n_check)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -pedantic)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(n_check_rrt
            src/Rrt.cpp
            src/Grasp.cpp)
target_link_libraries(n_check_rrt ${catkin_LIBRARIES})

add_executable(n_check_rrt_check test/check.cpp)
target_link_libraries(n_check_rrt_check n_check_rrt ${catkin_LIBRARIES})

install(TARGETS n_check_rrt n_check_rrt_check
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
