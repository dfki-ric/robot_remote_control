add_library(robot_remote_control-controlled_robot
            ControlledRobot.cpp ../TelemetryBuffer.cpp
)
target_link_libraries (robot_remote_control-controlled_robot robot_remote_control-types robot_remote_control-update_thread)
target_include_directories(robot_remote_control-controlled_robot
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control
	FILES_MATCHING PATTERN "*.hpp"
)

install (TARGETS
        robot_remote_control-controlled_robot
        EXPORT robot_remote_control-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

