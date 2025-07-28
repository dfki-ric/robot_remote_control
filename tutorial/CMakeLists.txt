cmake_minimum_required(VERSION 3.1.0)

function(build_tutorial_binary NAME)
    set(BINNAME ${NAME}_bin)

    add_executable(${BINNAME} ${NAME}.cpp)
    target_link_libraries(${BINNAME}
        robot_remote_control-controlled_robot
        robot_remote_control-robot_controller
        robot_remote_control-transport_zmq
    )
    target_include_directories(${BINNAME}
        PUBLIC
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/ControlledRobot>
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/RobotController>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )

endfunction(build_tutorial_binary)

build_tutorial_binary(tutorial1_ControlledRobot)
build_tutorial_binary(tutorial1_RobotController)

build_tutorial_binary(tutorial2_RobotController)
build_tutorial_binary(tutorial2_ControlledRobot)

build_tutorial_binary(tutorial3_RobotController)
build_tutorial_binary(tutorial3_ControlledRobot)

build_tutorial_binary(tutorial4_RobotController)
build_tutorial_binary(tutorial4_ControlledRobot)
