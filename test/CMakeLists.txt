
find_package (Threads)

rock_testsuite(test_suite suite.cpp
   test_Communication.cpp
   test_RingBuffer.cpp
   test_Thread.cpp
   DEPS robot_remote_control-controlled_robot
        robot_remote_control-robot_controller 
        robot_remote_control-transport_zmq
   LIBS ${CMAKE_THREAD_LIBS_INIT}
)
