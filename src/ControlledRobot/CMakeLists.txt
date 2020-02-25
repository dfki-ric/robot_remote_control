add_library(robot_remote_control-controlled_robot
            ControlledRobot.cpp ../TelemetryBuffer.cpp 
)
target_link_libraries (robot_remote_control-controlled_robot robot_remote_control-types robot_remote_control-update_thread)
target_include_directories(robot_remote_control-controlled_robot
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
		$<INSTALL_INTERFACE:include>
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
	DESTINATION include/robot_remote_control
	FILES_MATCHING PATTERN "*.hpp"
)


add_executable(robot_remote_control-controlled_robot_bin ControlledRobotMain.cpp)
target_link_libraries(robot_remote_control-controlled_robot_bin
    robot_remote_control-controlled_robot
    robot_remote_control-transport_zmq
)

install (TARGETS
        robot_remote_control-controlled_robot
        robot_remote_control-controlled_robot_bin
        EXPORT robot_remote_control-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

