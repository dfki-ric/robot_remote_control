find_package (Threads)

find_package(Boost REQUIRED COMPONENTS unit_test_framework system filesystem)

add_subdirectory(performance)

add_definitions(-DBOOST_TEST_DYN_LINK)

set (COMMON_SOURCE suite.cpp
            Transports.cpp
            test_Communication.cpp
            test_RingBuffer.cpp
            test_Thread.cpp
            test_SendTimers.cpp
            test_SimpleActionHelper.cpp
            test_TransportSupport.cpp
)
set (COMMON_LIBS
   ${CMAKE_THREAD_LIBS_INIT}
   robot_remote_control-transport_zmq
   robot_remote_control-controlled_robot
   robot_remote_control-robot_controller
   robot_remote_control-send_timers
   robot_remote_control-simple_action_helper
   ${Boost_LIBRARIES}
)

add_executable(test_suite ${COMMON_SOURCE} )
target_link_libraries(test_suite ${COMMON_LIBS})
target_compile_definitions(test_suite PUBLIC -DTRANSPORT_DEFAULT)
target_include_directories(test_suite
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>/src/Transports
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)


add_executable(test_suite_ipc ${COMMON_SOURCE} )
target_link_libraries(test_suite_ipc ${COMMON_LIBS})
target_compile_definitions(test_suite_ipc PUBLIC -DTRANSPORT_IPC)
target_include_directories(test_suite_ipc
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>/src/Transports
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)




if(ZLIB_FOUND)
    add_executable(test_suite_gzip ${COMMON_SOURCE})
    target_link_libraries(test_suite_gzip
      ${COMMON_LIBS}
      robot_remote_control-transport_wrapper_gzip
    )
    target_compile_definitions(test_suite_gzip PUBLIC -DTRANSPORT_DEFAULT_GZIP)
    target_include_directories(test_suite_gzip
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>/src/Transports
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>/src/TransportWrappers
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
endif()

if(UDT_FOUND)
    add_executable(test_suite_udt ${COMMON_SOURCE} test_UDT.cpp)
    target_link_libraries(test_suite_udt
      ${COMMON_LIBS}
      robot_remote_control-transport_udt
    )
    target_compile_definitions(test_suite_udt PUBLIC -DTRANSPORT_UDT)
    target_include_directories(test_suite_udt
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>/src/Transports
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
endif()

if(websocketpp_FOUND)
    add_executable(test_suite_ws ${COMMON_SOURCE})
    target_link_libraries(test_suite_ws
      ${COMMON_LIBS}
      robot_remote_control-transport_ws
    )
    target_compile_definitions(test_suite_ws PUBLIC -DTRANSPORT_WS)
    add_executable(test_suite_ws_json ${COMMON_SOURCE})
    target_link_libraries(test_suite_ws_json
      ${COMMON_LIBS}
      robot_remote_control-transport_ws
    )
    target_compile_definitions(test_suite_ws_json PUBLIC -DTRANSPORT_WS_JSON)
endif()

if(websocketpp_FOUND AND cpprestsdk_FOUND)
    add_executable(test_suite_web ${COMMON_SOURCE})
    target_link_libraries(test_suite_web
      ${COMMON_LIBS}
      robot_remote_control-transport_ws
      robot_remote_control-transport_http
    )
    target_compile_definitions(test_suite_web PUBLIC -DTRANSPORT_WEB)
endif()


#set_target_properties(test_suite PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
