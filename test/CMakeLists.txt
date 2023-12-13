find_package (Threads)

find_package(Boost REQUIRED COMPONENTS unit_test_framework system filesystem)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

add_subdirectory(performance)

add_definitions(-DBOOST_TEST_DYN_LINK)

set (COMMON_SOURCE suite.cpp
            test_Communication.cpp
            test_RingBuffer.cpp
            test_Thread.cpp
            test_SendTimers.cpp
)
set (COMMON_LIBS
   ${CMAKE_THREAD_LIBS_INIT}
   robot_remote_control-transport_zmq
   robot_remote_control-controlled_robot
   robot_remote_control-robot_controller
   robot_remote_control-send_timers
   ${Boost_LIBRARIES}
)

add_executable(test_suite ${COMMON_SOURCE} )
target_link_libraries(test_suite ${COMMON_LIBS})
target_compile_definitions(test_suite PUBLIC -DTRANSPORT_DEFAULT)

add_executable(test_suite_ipc ${COMMON_SOURCE} )
target_link_libraries(test_suite_ipc ${COMMON_LIBS})
target_compile_definitions(test_suite_ipc PUBLIC -DTRANSPORT_IPC)

if(ZLIB_FOUND)
    add_executable(test_suite_gzip ${COMMON_SOURCE})
    target_link_libraries(test_suite_gzip
      ${COMMON_LIBS}
      robot_remote_control-transport_wrapper_gzip
    )
    target_compile_definitions(test_suite_gzip PUBLIC -DTRANSPORT_DEFAULT_GZIP)
endif()

if(UDT_FOUND)
    add_executable(test_suite_udt ${COMMON_SOURCE} test_UDT.cpp)
    target_link_libraries(test_suite_udt
      ${COMMON_LIBS}
      robot_remote_control-transport_udt
    )
    target_compile_definitions(test_suite_udt PUBLIC -DTRANSPORT_UDT)
endif()

#set_target_properties(test_suite PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
