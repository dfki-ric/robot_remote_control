find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

add_library(robot_remote_control-transport_http
    RestServer.cpp
    TransportHttp.cpp
)
target_include_directories(robot_remote_control-transport_http
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries (robot_remote_control-transport_http
    cpprestsdk::cpprest
    Boost::filesystem
    ${CMAKE_THREAD_LIBS_INIT}
)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/Transports/UDT/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control/Transports
	FILES_MATCHING PATTERN "*.hpp"
)
install (TARGETS robot_remote_control-transport_http
    EXPORT robot_remote_control-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)