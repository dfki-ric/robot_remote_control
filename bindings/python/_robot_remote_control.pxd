
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.memory cimport shared_ptr
from libc.stdint cimport uint16_t

cdef extern from "Transports/Transport.hpp" namespace "robot_remote_control":
    cdef cppclass Transport:
        enum Flags:
            NONE = 0x0,
            NOBLOCK = 0x1
        Transport()
   
    ctypedef shared_ptr[Transport] TransportSharedPtr


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

        void printConnections()


cdef extern from "RobotController/RobotControllerWrapper.hpp" namespace "robot_remote_control":
    cdef cppclass RobotControllerWrapper:
        RobotControllerWrapper (RobotController *controller)
        bool getCurrentPose(string *buf)


cdef extern from "RobotController/RobotController.hpp" namespace "robot_remote_control":
    cdef cppclass RobotController:
        RobotController (TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport, const size_t &buffersize = 10, const float &maxLatency = 1)

        void update()
        void startUpdateThread(const unsigned int &milliseconds)

