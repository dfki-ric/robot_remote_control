# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.1.0)

project(robot_remote_control VERSION "0.0.9")

set(BUILD_SHARED_LIBS ON)
 
######################################################## CHECK FOR UDT
find_package(PkgConfig)
pkg_check_modules(UDT "udt")
if(UDT_FOUND)
    message(STATUS "  Found UDT using pkg-config: ${UDT_LIBRARIES}")
else()
    find_library(UDT NAMES "udt" HINTS "/usr/lib")
    if(UDT)
        set(UDT_FOUND true)
    	set(UDT_LIBRARIES ${UDT})
        message(STATUS "  Found UDT library: ${UDT_LIBRARIES}")
    else()
        add_definitions(-DTRANSPORT_UDT_UNSUPPORTED)
        message(STATUS "    could not find UDT library\n" 
                       "       -- compiling without out UDT support\n"
                       "       -- apt install libudt-dev or build udt from source")
    endif()
endif()

if( RRC_STATISTICS )
    add_definitions(-DRRC_STATISTICS)
endif( RRC_STATISTICS )


################################################### ADD SUBDIRECTORIES
add_subdirectory(src)

if( BUILD_EXAMPLES )
    message("Adding example directory") 
    add_subdirectory(examples)
endif( BUILD_EXAMPLES )

#set (BUILD_TESTS ON)
if( BUILD_TESTS )
    message("Adding test directory") 
    add_subdirectory(test)
endif( BUILD_TESTS )
