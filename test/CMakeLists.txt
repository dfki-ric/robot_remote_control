
find_package (Threads)

add_executable(test_suite
   suite.cpp
   test_Communication.cpp
   test_RingBuffer.cpp
   test_Thread.cpp
)

set(ENV{PKG_CONFIG_PATH}  "${CMAKE_BINARY_DIR}/src")

find_package(PkgConfig REQUIRED) 
pkg_check_modules(REMOTE REQUIRED
   robot_remote_control-types
   robot_remote_control-controlled_robot
   robot_remote_control-robot_controller 
   robot_remote_control-transport_zmq
)


message(${REMOTE_LIBRARIES})

find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS unit_test_framework system)

add_definitions(-DBOOST_TEST_DYN_LINK)

target_link_libraries(test_suite
   ${CMAKE_THREAD_LIBS_INIT}
   ${REMOTE_LIBRARIES}
   ${Protobuf_LIBRARIES}
   ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)


set_target_properties(test_suite PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
