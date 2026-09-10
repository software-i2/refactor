cmake_minimum_required(VERSION 3.0.2)
project(n_kine)

# Pure maths: catkin only packages it, nothing here links ROS. n_conf brings the
# config reader, which is yaml-cpp and no more.
find_package(catkin REQUIRED COMPONENTS n_conf)

catkin_package(INCLUDE_DIRS include
               LIBRARIES n_kine
               CATKIN_DEPENDS n_conf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -pedantic)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(n_kine
            src/Geom.cpp
            src/Fk.cpp
            src/Ik.cpp)
target_link_libraries(n_kine ${catkin_LIBRARIES})

# Also runs outside Docker:
#   g++ -std=c++17 -Iinclude -I../n_conf/include test/check.cpp src/*.cpp \
#       ../n_conf/src/Doc.cpp -lyaml-cpp -o /tmp/check
add_executable(n_kine_check test/check.cpp)
target_link_libraries(n_kine_check n_kine ${catkin_LIBRARIES})

install(TARGETS n_kine n_kine_check
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
