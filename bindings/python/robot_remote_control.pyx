# distutils: language = c++

#https://stackoverflow.com/questions/47005382/cython-cannot-convert-python-object-error

from libcpp.string cimport string
from libcpp.memory cimport shared_ptr
from cython.operator cimport dereference as deref
#from cython.operator cimport reference as ref
from _robot_remote_control cimport TransportZmq
from _robot_remote_control cimport Transport
from _robot_remote_control cimport RobotController
from _robot_remote_control cimport TransportSharedPtr

#cdef class Transport:
#    cdef shared_ptr[_Transport] thisptr
#    
#    def __cinit__ (self, transport):
#        self.thisptr.reset(<_Transport *> transport.thisptr.get())

cdef class PyTransportSharedPtr:
    cdef TransportSharedPtr* thisptr

    def __cinit__(self):
        self.thisptr = new TransportSharedPtr()

    def reset(self, PyTransportZmq trans):
        deref(self.thisptr).reset( <Transport*> (trans.thisptr))


cdef class PyTransportZmq:
    cdef TransportZmq *thisptr
    
    def __cinit__(self, addr, type):
        self.thisptr = new TransportZmq(str.encode(addr),type)
        
    def __dealloc__(self):
       del self.thisptr


    def send(self, msg):
        #encode needs to go to support binary
        return deref(self.thisptr).send(str.encode(msg))

    def receive(self):
        cdef string* buf = new string()
        recvsize = deref(self.thisptr).receive(buf)
        print(deref(buf))
        msg = deref(buf).decode("utf-8")
        del buf
        return msg
        

cdef class PyRobotController:
    cdef RobotController* thisptr
    cdef PyTransportZmq telemetry
    cdef PyTransportSharedPtr transportptr
    #def __cinit__(self, transport):
       
        #transportptr.reset( transport.getPtr() )

        #self.thisptr = new RobotController(transportptr)
        
    def setTelemetryTransport(self, PyTransportZmq transport):
        telemetry = transport
        self.transportptr.reset( telemetry )


    cdef init(self):
        self.thisptr = new RobotController(deref(self.transportptr.thisptr))

    def __dealloc__(self):
       del self.thisptr
