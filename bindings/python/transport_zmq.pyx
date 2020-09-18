from libcpp.string cimport string
from libcpp.memory cimport shared_ptr
from cython.operator cimport dereference as deref
#from cython.operator cimport reference as ref
from _transport_zmq cimport TransportZmq as _TransportZmq

cdef class TransportZmq:
    cdef shared_ptr[_TransportZmq] thisptr
    
    def __cinit__(self, addr, type):
        self.thisptr.reset(new _TransportZmq(str.encode(addr),type))

    def send(self, msg):
        #encode needs to go to support binary
        return deref(self.thisptr.get()).send(str.encode(msg))

    def receive(self):
        cdef string* buf = new string()
        recvsize = deref(self.thisptr.get()).receive(buf)
        print(deref(buf))
        return deref(buf).decode("utf-8")