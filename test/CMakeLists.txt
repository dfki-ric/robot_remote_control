
find_package (Threads)

rock_testsuite(test_suite suite.cpp
   test_Communication.cpp
   test_RingBuffer.cpp
   test_Thread.cpp
   DEPS interaction-library-controlled_robot
        interaction-library-robot_controller 
        interaction-library-controlled_robot_transport_zmq
   LIBS ${CMAKE_THREAD_LIBS_INIT}
)
