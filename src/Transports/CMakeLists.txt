################################################################# ZMQ
if (ZMQ_FOUND)
    add_subdirectory(ZeroMQ)
endif()

################################################################# UDT
if(UDT_FOUND)
    add_subdirectory(UDT)
endif()


if (websocketpp_FOUND)
    add_subdirectory(WebSocket)
endif()

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/Transport.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control/Transports
)




