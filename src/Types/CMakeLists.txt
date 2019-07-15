# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.

cmake_minimum_required(VERSION 3.1.0)

find_package(Protobuf REQUIRED)

if (CMAKE_MINOR_VERSION LESS 6)
	set(Protobuf_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS})
	set(Protobuf_LIBRARIES ${PROTOBUF_LIBRARIES})
	set(Protobuf_PROTOC_LIBRARIES ${PROTOBUF_PROTOC_LIBRARIES})
endif()

include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_custom_target(typegen_cpp COMMAND protoc --cpp_out=${CMAKE_CURRENT_SOURCE_DIR} controlledRobot.proto WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_custom_target(typegen_js COMMAND protoc --js_out=${CMAKE_CURRENT_SOURCE_DIR} controlledRobot.proto WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_custom_target(typegen_python COMMAND protoc --python_out=${CMAKE_CURRENT_SOURCE_DIR} controlledRobot.proto WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})


INSTALL(FILES controlledRobot.proto DESTINATION protobuf)
