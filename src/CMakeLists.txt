rock_library(interaction-library-controlled_robot
    SOURCES ControlledRobot.cpp
    HEADERS ControlledRobot.hpp
    DEPS_PKGCONFIG interaction-types libzmq)

rock_library(interaction-library-robot_controller
    SOURCES RobotController.cpp
    HEADERS RobotController.hpp
    DEPS_PKGCONFIG interaction-types libzmq)

rock_executable(interaction-library-robot_controller_bin ControllerMain.cpp
    DEPS interaction-library-robot_controller)

rock_executable(interaction-library-controlled_robot_bin RobotMain.cpp
    DEPS interaction-library-controlled_robot)


