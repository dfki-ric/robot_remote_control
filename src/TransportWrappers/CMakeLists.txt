
################################################################# Gzip wrapper
if(ZLIB_FOUND)
    add_subdirectory(Gzip)
endif()


install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/TransportWrapper.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control/Transports
)