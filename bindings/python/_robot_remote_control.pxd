
from libcpp.string cimport string, int
from libcpp.memory cimport shared_ptr

cdef extern from "Transports/Transport.hpp" namespace "robot_remote_control":
    cdef cppclass Transport:
        Transport()
   
    cdef cppclass TransportSharedPtr:
        TransportSharedPtr()
        void reset(Transport* ptr)

cdef extern from "Transports/TransportZmq.hpp" namespace "robot_remote_control":
    cdef cppclass TransportZmq:
        enum ConnectionType:
            REQ = 0,
            REP = 1,
            PUB = 2,
            SUB = 3
        TransportZmq(string addr, ConnectionType type)

        int send(string msg)

        int receive(string* msg)



cdef extern from "RobotController/RobotController.hpp" namespace "robot_remote_control":
    cdef cppclass RobotController:
        RobotController (TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport = TransportSharedPtr(), const size_t &buffersize = 10, const float &maxLatency = 1)
