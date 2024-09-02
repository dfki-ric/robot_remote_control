cmake_minimum_required(VERSION 3.1.0)

set(CURSES_NEED_NCURSES true)
find_package(Curses)

if (CURSES_HAVE_NCURSES_H)
    add_executable(virtual_joystick VirtualJoystick.cpp)
    target_link_libraries(virtual_joystick
        ncurses
        robot_remote_control-robot_controller
        robot_remote_control-transport_zmq
    )
    target_include_directories(virtual_joystick
        PUBLIC
            $<BUILD_INTERFACE:/usr/include/readline>
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/RobotController>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
    install (TARGETS virtual_joystick
             EXPORT robot_remote_control-targets
             RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif(CURSES_HAVE_NCURSES_H)

add_executable(robot_controller RobotControllerCLI.cpp ConsoleCommands.cpp)
target_link_libraries(robot_controller
    robot_remote_control-robot_controller
    robot_remote_control-transport_zmq
    readline
)
target_include_directories(robot_controller
	PUBLIC
        $<BUILD_INTERFACE:/usr/include/readline>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/RobotController>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)


install (TARGETS robot_controller
         EXPORT robot_remote_control-targets
         RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
