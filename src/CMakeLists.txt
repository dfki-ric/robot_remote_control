
rock_library(interaction-library-update_thread
    SOURCES UpdateThread/UpdateThread.cpp
    HEADERS UpdateThread/UpdateThread.hpp UpdateThread/ThreadProtectedVar.hpp
)

rock_library(interaction-library-controlled_robot
    SOURCES ControlledRobot.cpp
    HEADERS ControlledRobot.hpp MessageTypes.hpp RingBuffer.hpp Transports/Transport.hpp
    DEPS interaction-library-update_thread
    DEPS_PKGCONFIG interaction-types protobuf)

rock_library(interaction-library-robot_controller
    SOURCES RobotController.cpp
    HEADERS RobotController.hpp MessageTypes.hpp RingBuffer.hpp Transports/Transport.hpp
    DEPS interaction-library-update_thread
    DEPS_PKGCONFIG interaction-types protobuf)

rock_library(interaction-library-controlled_robot_transport_zmq
    SOURCES Transports/TransportZmq.cpp
    HEADERS Transports/TransportZmq.hpp Transports/Transport.hpp
    DEPS_PKGCONFIG libzmq)

rock_executable(interaction-library-robot_controller_bin RobotControllerMain.cpp
    DEPS interaction-library-robot_controller interaction-library-controlled_robot_transport_zmq)

rock_executable(interaction-library-controlled_robot_bin ControlledRobotMain.cpp
    DEPS interaction-library-controlled_robot interaction-library-controlled_robot_transport_zmq)


