
add_library(robot_remote_control-transport_zmq 
            TransportZmq.cpp
)
target_include_directories(robot_remote_control-transport_zmq 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries (robot_remote_control-transport_zmq ${ZMQ_LIBRARIES})

# install with the view of the top folder (no /ZeroMQ/ for include path)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/Transports/ZeroMQ/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control/Transports
	FILES_MATCHING PATTERN "*.hpp"
)

install (TARGETS robot_remote_control-transport_zmq
         EXPORT robot_remote_control-targets
         LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
         RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

## Proxy Server

find_package(Threads REQUIRED)
add_executable(robot_remote_control-zmq_proxy ZMQProxyMain.cpp)
target_link_libraries(robot_remote_control-zmq_proxy
    ${ZMQ_LIBRARIES}
    Threads::Threads
)

install (TARGETS robot_remote_control-zmq_proxy
         EXPORT robot_remote_control-zmq_proxy-targets
         RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

