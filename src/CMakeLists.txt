

find_package(Protobuf REQUIRED)

set(Protobuf_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS})
set(Protobuf_LIBRARIES ${PROTOBUF_LIBRARIES})
set(Protobuf_PROTOC_LIBRARIES ${PROTOBUF_PROTOC_LIBRARIES})

include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# add_custom_target(typegen_cpp 
#                     DEPENDS Types/controlledRobot.proto
#                     SOURCES Types/controlledRobot.proto
#                     COMMAND protoc --cpp_out=${CMAKE_CURRENT_SOURCE_DIR}/Types controlledRobot.proto WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Types
#                     )

add_custom_command( OUTPUT Types/controlledRobot.pb.cc Types/controlledRobot.pb.h
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Types
                    COMMAND protoc --cpp_out=${CMAKE_CURRENT_SOURCE_DIR}/Types controlledRobot.proto
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Types/controlledRobot.pb.cc ${CMAKE_CURRENT_BINARY_DIR}/Types/controlledRobot.pb.cc 
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Types/controlledRobot.pb.h ${CMAKE_CURRENT_BINARY_DIR}/Types/controlledRobot.pb.h
                    MAIN_DEPENDENCY Types/controlledRobot.proto
                    )

#add_custom_target(typegen_js COMMAND protoc --js_out=${CMAKE_CURRENT_SOURCE_DIR} controlledRobot.proto WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
#add_custom_tainclude_directories(${CMAKE_CURRENT_BINARY_DIR})rget(typegen_python COMMAND protoc --python_out=${CMAKE_CURRENT_SOURCE_DIR} controlledRobot.proto WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

INSTALL(FILES  Types/controlledRobot.proto DESTINATION protobuf)


# set_source_files_properties(
#     ${CMAKE_CURRENT_SOURCE_DIR}/Types/controlledRobot.pb.cc
#     ${CMAKE_CURRENT_SOURCE_DIR}/Types/controlledRobot.pb.h
#     PROPERTIES GENERATED TRUE
# )



rock_library(interaction-library-update_thread
    SOURCES UpdateThread/UpdateThread.cpp
    HEADERS UpdateThread/UpdateThread.hpp UpdateThread/ThreadProtectedVar.hpp
)

rock_library(interaction-library-types
    SOURCES Types/controlledRobot.pb.cc
    HEADERS Types/controlledRobot.pb.h
    DEPS_PKGCONFIG protobuf
)

rock_library(interaction-library-controlled_robot
    SOURCES ControlledRobot.cpp TelemetryBuffer.cpp 
    HEADERS ControlledRobot.hpp MessageTypes.hpp RingBuffer.hpp TelemetryBuffer.hpp Transports/Transport.hpp
    DEPS interaction-library-update_thread interaction-library-types
    )

rock_library(interaction-library-robot_controller
    SOURCES RobotController.cpp TelemetryBuffer.cpp
    HEADERS RobotController.hpp MessageTypes.hpp RingBuffer.hpp TelemetryBuffer.hpp Transports/Transport.hpp
    DEPS interaction-library-update_thread interaction-library-types
    )

#make sure protobuf runs first
#add_dependencies(interaction-library-controlled_robot typegen_cpp)
#add_dependencies(interaction-library-robot_controller typegen_cpp)


rock_library(interaction-library-controlled_robot_transport_zmq
    SOURCES Transports/TransportZmq.cpp
    HEADERS Transports/TransportZmq.hpp Transports/Transport.hpp
    DEPS_PKGCONFIG libzmq)

rock_executable(interaction-library-robot_controller_bin RobotControllerMain.cpp
    DEPS interaction-library-robot_controller interaction-library-controlled_robot_transport_zmq)

rock_executable(interaction-library-controlled_robot_bin ControlledRobotMain.cpp
    DEPS interaction-library-controlled_robot interaction-library-controlled_robot_transport_zmq)


