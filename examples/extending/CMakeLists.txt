# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.1.0)

project(extended_remote_control VERSION "0.0.9")

set(BUILD_SHARED_LIBS ON)

add_subdirectory(src)

set(BUILD_EXAMPLES ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif(BUILD_EXAMPLES)

set(BUILD_TESTS ON)
if(BUILD_TESTS)
    add_subdirectory(test)
endif(BUILD_TESTS)
