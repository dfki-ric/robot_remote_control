# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.1.0)

#find_package(Rock)
#rock_init(robot_remote_control 0.1)
#rock_standard_layout()

project(robot_remote_control VERSION "0.0.9")

set(BUILD_SHARED_LIBS ON)

add_subdirectory(src)

if( BUILD_EXAMPLES )
    message("Adding example directory") 
    add_subdirectory(examples)
endif( BUILD_EXAMPLES )

if( BUILD_TESTS )
    message("Adding test directory") 
    add_subdirectory(test)
endif( BUILD_TESTS )


add_subdirectory(bindings/python)