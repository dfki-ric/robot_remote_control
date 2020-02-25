
find_package(PkgConfig)
pkg_check_modules(ZMQ REQUIRED libzmq)

add_library(robot_remote_control-transport_zmq 
            TransportZmq.cpp
)
target_include_directories(robot_remote_control-transport_zmq 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)
target_link_libraries (robot_remote_control-transport_zmq ${ZMQ_LIBRARIES})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Transports
	DESTINATION include/robot_remote_control
	FILES_MATCHING PATTERN "*.hpp"
)


install (TARGETS robot_remote_control-transport_zmq
        EXPORT robot_remote_control-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
