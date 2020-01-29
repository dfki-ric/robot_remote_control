

find_package(Protobuf REQUIRED)

set(CMAKE_BUILD_TYPE Release)

set(Protobuf_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS})
set(Protobuf_LIBRARIES ${PROTOBUF_LIBRARIES})
set(Protobuf_PROTOC_LIBRARIES ${PROTOBUF_PROTOC_LIBRARIES})

include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_custom_command( OUTPUT Types/RobotRemoteControl.pb.cc Types/RobotRemoteControl.pb.h
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Types
                    COMMAND protoc --cpp_out=${CMAKE_CURRENT_SOURCE_DIR}/Types RobotRemoteControl.proto
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Types/RobotRemoteControl.pb.cc ${CMAKE_CURRENT_BINARY_DIR}/Types/RobotRemoteControl.pb.cc 
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Types/RobotRemoteControl.pb.h ${CMAKE_CURRENT_BINARY_DIR}/Types/RobotRemoteControl.pb.h
                    MAIN_DEPENDENCY Types/RobotRemoteControl.proto
                    )

include_directories(${CMAKE_CURRENT_BINARY_DIR})

INSTALL(FILES  Types/RobotRemoteControl.proto DESTINATION protobuf)


rock_library(robot_remote_control-update_thread
    SOURCES UpdateThread/UpdateThread.cpp
    HEADERS UpdateThread/UpdateThread.hpp UpdateThread/ThreadProtectedVar.hpp
)

rock_library(robot_remote_control-types
    SOURCES Types/RobotRemoteControl.pb.cc
    HEADERS Types/RobotRemoteControl.pb.h
    DEPS_PKGCONFIG protobuf
)

rock_library(robot_remote_control-controlled_robot
    SOURCES ControlledRobot.cpp TelemetryBuffer.cpp 
    HEADERS ControlledRobot.hpp MessageTypes.hpp RingBuffer.hpp TelemetryBuffer.hpp Transports/Transport.hpp
    DEPS robot_remote_control-update_thread robot_remote_control-types
    )

rock_library(robot_remote_control-robot_controller
    SOURCES RobotController.cpp TelemetryBuffer.cpp SimpleSensorBuffer.cpp
    HEADERS RobotController.hpp MessageTypes.hpp RingBuffer.hpp TelemetryBuffer.hpp SimpleSensorBuffer.hpp Transports/Transport.hpp 
    DEPS robot_remote_control-update_thread robot_remote_control-types
    )

rock_library(robot_remote_control-transport_zmq
    SOURCES Transports/TransportZmq.cpp
    HEADERS Transports/TransportZmq.hpp Transports/Transport.hpp
    DEPS_PKGCONFIG libzmq)

rock_executable(robot_remote_control-robot_controller_bin RobotControllerMain.cpp
    DEPS robot_remote_control-robot_controller robot_remote_control-transport_zmq)

rock_executable(robot_remote_control-controlled_robot_bin ControlledRobotMain.cpp
    DEPS robot_remote_control-controlled_robot robot_remote_control-transport_zmq)


