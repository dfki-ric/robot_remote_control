################################################################# ZMQ
find_package(PkgConfig)
pkg_check_modules(ZMQ REQUIRED libzmq)

add_library(robot_remote_control-transport_zmq 
            TransportZmq.cpp
)
target_include_directories(robot_remote_control-transport_zmq 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries (robot_remote_control-transport_zmq ${ZMQ_LIBRARIES})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control
	FILES_MATCHING PATTERN "*.hpp"
)


install (TARGETS robot_remote_control-transport_zmq
         EXPORT robot_remote_control-targets
         LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
         RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

################################################################# UDT
if(UDT_FOUND)
    find_package(Threads REQUIRED)
    add_library(robot_remote_control-transport_udt
                TransportUDT.cpp
    )
    target_include_directories(robot_remote_control-transport_udt
    	PUBLIC
    		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
    target_link_libraries (robot_remote_control-transport_udt
                           ${UDT}
                           ${CMAKE_THREAD_LIBS_INIT}
    )
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control
            FILES_MATCHING PATTERN "*.hpp"
    )
    install (TARGETS robot_remote_control-transport_udt
             EXPORT robot_remote_control-targets
             LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
             RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

################################################################# Gzip wrapper
if(ZLIB_FOUND)
    add_library(robot_remote_control-transport_wrapper_gzip
                TransportWrapperGzip.cpp
    )
    target_include_directories(robot_remote_control-transport_wrapper_gzip
    	PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Wrappers>
    		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
    target_link_libraries (robot_remote_control-transport_wrapper_gzip
                            ZLIB::ZLIB
    )
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Wrappers
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control
            FILES_MATCHING PATTERN "*.hpp"
    )
    install (TARGETS robot_remote_control-transport_wrapper_gzip
             EXPORT robot_remote_control-targets
             LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
             RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

set(CMAKE_BUILD_TYPE Debug)

add_executable(robot_remote_control-zmq_telemetry_proxy ZMQProxyMain.cpp)
target_link_libraries(robot_remote_control-zmq_telemetry_proxy
    ${ZMQ_LIBRARIES}
)

install (TARGETS robot_remote_control-zmq_telemetry_proxy
         EXPORT robot_remote_control-zmq_telemetry_proxy-targets
         RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

