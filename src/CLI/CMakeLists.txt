cmake_minimum_required(VERSION 3.1.0)

set(CURSES_NEED_NCURSES true)
find_package(Curses)
find_package(Boost COMPONENTS program_options REQUIRED)

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

if (websocketpp_FOUND)
    target_compile_definitions(robot_controller PRIVATE -DADD_WEBSOCKET_TRANSPORT=TRUE)
    set(OTHER_TRANSPORT_LIBS robot_remote_control-transport_ws)
endif()

target_link_libraries(robot_controller
    robot_remote_control-robot_controller
    robot_remote_control-transport_zmq
    ${OTHER_TRANSPORT_LIBS}
    readline
    Boost::program_options
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



find_package(OpenCV)

if (OpenCV_FOUND)

    add_executable(robot_controller_camera_viewer CameraViewer.cpp)
    INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
    target_link_libraries(robot_controller_camera_viewer
        robot_remote_control-robot_controller
        robot_remote_control-transport_zmq
        ${OpenCV_LIBS}
    )
    target_include_directories(robot_controller_camera_viewer
        PUBLIC
            $<BUILD_INTERFACE:/usr/include/readline>
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/RobotController>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )


    install (TARGETS robot_controller_camera_viewer
            EXPORT robot_remote_control-targets
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

endif(OpenCV_FOUND)
