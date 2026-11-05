
add_library(robot_remote_control-transport_proxy
    GenericProxy.cpp
)
target_include_directories(robot_remote_control-transport_proxy
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(robot_remote_control-transport_proxy
    robot_remote_control-types
    robot_remote_control-update_thread
)
install(FILES GenericProxy.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control/TransportProxy
)
install(TARGETS robot_remote_control-transport_proxy
    EXPORT robot_remote_control-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# connects to robots via zmq and hosts the web interface
if (TARGET robot_remote_control-transport_http AND TARGET robot_remote_control-transport_ws)
    find_package(Boost COMPONENTS program_options REQUIRED)
    add_executable(robot_remote_control-web_proxy WebProxyMain.cpp)
    target_link_libraries(robot_remote_control-web_proxy
        robot_remote_control-transport_proxy
        robot_remote_control-robot_controller
        robot_remote_control-transport_zmq
        robot_remote_control-transport_http
        robot_remote_control-transport_ws
        Boost::program_options
        stdc++fs
    )
    target_compile_definitions(robot_remote_control-web_proxy PRIVATE
        RRC_WEB_INSTALL_DIR="${CMAKE_INSTALL_FULL_DATADIR}/robot_remote_control/web"
        RRC_WEB_SOURCE_DIR="${PROJECT_SOURCE_DIR}/src/Web"
    )
    install(TARGETS robot_remote_control-web_proxy
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
else()
    message(STATUS "no http/websocket transports, building without robot_remote_control-web_proxy")
endif()
