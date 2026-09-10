cmake_minimum_required(VERSION 3.0.2)
project(n_check)

# Pure geometry: catkin only packages it, nothing here links ROS.
find_package(catkin REQUIRED COMPONENTS n_kine n_conf)

catkin_package(INCLUDE_DIRS include
               LIBRARIES n_check
               CATKIN_DEPENDS n_kine n_conf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -pedantic)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(n_check
            src/Field.cpp
            src/Body.cpp
            src/Grasp.cpp
            src/Scene.cpp)
target_link_libraries(n_check ${catkin_LIBRARIES})

# Also runs outside Docker; pass a field file to exercise the loader.
add_executable(n_check_check test/check.cpp)
target_link_libraries(n_check_check n_check ${catkin_LIBRARIES})

install(TARGETS n_check n_check_check
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
