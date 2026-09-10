cmake_minimum_required(VERSION 3.0.2)
project(n_conf)

# Strict config reading: catkin only packages it, nothing here links ROS.
find_package(catkin REQUIRED)

# Melodic ships yaml-cpp 0.5, whose config file sets YAML_CPP_LIBRARIES. Newer
# ones export a target of the same name instead, so fall back to the bare name
# and let the linker find libyaml-cpp.
find_package(yaml-cpp REQUIRED)
if(NOT YAML_CPP_LIBRARIES)
    set(YAML_CPP_LIBRARIES yaml-cpp)
endif()

catkin_package(INCLUDE_DIRS include
               LIBRARIES n_conf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -pedantic)

include_directories(include ${YAML_CPP_INCLUDE_DIR})

add_library(n_conf src/Doc.cpp)
target_link_libraries(n_conf ${YAML_CPP_LIBRARIES})

install(TARGETS n_conf
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
