add_library(robot_remote_control-send_timers
            SendTimers.cpp
            )

target_include_directories(robot_remote_control-send_timers
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install (TARGETS robot_remote_control-send_timers
        EXPORT robot_remote_control-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(ZLIB_FOUND)
       
        add_library(robot_remote_control-compression
                Compression.cpp
                )

        target_compile_definitions(robot_remote_control-compression PUBLIC -DZLIB_FOUND)

        target_include_directories(robot_remote_control-compression
                PUBLIC
                        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        )

        install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control
                FILES_MATCHING PATTERN "*.hpp"
        )

        install (TARGETS robot_remote_control-compression
                EXPORT robot_remote_control-targets
                LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
                RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )
endif()


add_library(robot_remote_control-simple_action_helper
            SimpleActionHelper.cpp
            )

target_include_directories(robot_remote_control-simple_action_helper
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install (TARGETS robot_remote_control-simple_action_helper
        EXPORT robot_remote_control-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control
	FILES_MATCHING PATTERN "*.hpp"
)
