rock_testsuite(test_suite suite.cpp
   test_Dummy.cpp
   test_RingBuffer.cpp
   DEPS interaction-library-controlled_robot
        interaction-library-robot_controller 
        interaction-library-controlled_robot_transport_zmq
)
