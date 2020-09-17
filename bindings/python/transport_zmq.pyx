from libcpp.string cimport string
from libcpp.memory cimport shared_ptr
from cython.operator cimport dereference as deref
from _transport_zmq cimport TransportZmq as _TransportZmq

cdef class TransportZmq:
    cdef shared_ptr[_TransportZmq] thisptr
    
    def __init__(self, addr, type):
        self.thisptr.reset(new _TransportZmq(addr,type))

#    def send(self, string msg):
#        return deref(self.thisptr.get()).send(msg)
