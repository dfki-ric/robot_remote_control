
from libcpp.string cimport string, int

cdef extern from "TransportZmq.hpp" namespace "robot_remote_control":
    cdef cppclass TransportZmq:
        enum ConnectionType:
            REQ = 0,
            REP = 1,
            PUB = 2,
            SUB = 3
        TransportZmq(string addr, ConnectionType type)
        send(string msg)