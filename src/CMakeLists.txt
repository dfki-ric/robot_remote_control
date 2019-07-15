
add_subdirectory(Types)

rock_library(interaction-library-update_thread
    SOURCES UpdateThread/UpdateThread.cpp
    HEADERS UpdateThread/UpdateThread.hpp UpdateThread/ThreadProtectedVar.hpp
)

rock_library(interaction-library-controlled_robot
    SOURCES ControlledRobot.cpp Types/controlledRobot.pb.cc
    HEADERS ControlledRobot.hpp MessageTypes.hpp RingBuffer.hpp Transports/Transport.hpp Types/controlledRobot.pb.h
    DEPS interaction-library-update_thread
    DEPS_PKGCONFIG protobuf)

rock_library(interaction-library-robot_controller
    SOURCES RobotController.cpp Types/controlledRobot.pb.cc
    HEADERS RobotController.hpp MessageTypes.hpp RingBuffer.hpp Transports/Transport.hpp Types/controlledRobot.pb.h
    DEPS interaction-library-update_thread
    DEPS_PKGCONFIG protobuf)

#make sure protobuf runs first
add_dependencies(interaction-library-controlled_robot typegen_cpp)
add_dependencies(interaction-library-robot_controller typegen_cpp)



rock_library(interaction-library-controlled_robot_transport_zmq
    SOURCES Transports/TransportZmq.cpp
    HEADERS Transports/TransportZmq.hpp Transports/Transport.hpp
    DEPS_PKGCONFIG libzmq)

rock_executable(interaction-library-robot_controller_bin RobotControllerMain.cpp
    DEPS interaction-library-robot_controller interaction-library-controlled_robot_transport_zmq)

rock_executable(interaction-library-controlled_robot_bin ControlledRobotMain.cpp
    DEPS interaction-library-controlled_robot interaction-library-controlled_robot_transport_zmq)


