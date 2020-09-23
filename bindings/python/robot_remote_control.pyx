# distutils: language = c++

#https://stackoverflow.com/questions/47005382/cython-cannot-convert-python-object-error

from libcpp.string cimport string
from libcpp.memory cimport shared_ptr
from cython.operator cimport dereference as deref

from _robot_remote_control cimport Transport
from _robot_remote_control cimport TransportZmq
from _robot_remote_control cimport RobotController
from _robot_remote_control cimport RobotControllerWrapper
import RobotRemoteControl_pb2 as types

cdef class PyTransportZmq:
    cdef TransportZmq *thisptr
    cdef shared_ptr[Transport] shared
    
    def __cinit__(self, addr, type):
        self.thisptr = new TransportZmq(str.encode(addr),type)
        self.shared = shared_ptr[Transport](self.thisptr)

    def printConnections(self):
        deref(self.thisptr).printConnections()

    def send(self, msg):
        #encode needs to go to support binary
        return deref(self.thisptr).send(msg)

    def receive(self):
        cdef string* buf = new string()
        recvsize = deref(self.thisptr).receive(buf)
        print recvsize
        print(deref(buf))
        msg = deref(buf)
        del buf
        return msg


cdef class PyRobotController:
    cdef RobotController* thisptr
    cdef RobotControllerWrapper* wrapper

    def __cinit__(self, PyTransportZmq commands, PyTransportZmq telemetry):
        self.thisptr = new RobotController(commands.shared, telemetry.shared, 10, 1)    
        self.wrapper = new RobotControllerWrapper(self.thisptr)

    def __dealloc__(self):
        del self.wrapper
        del self.thisptr

    def update(self):
        deref(self.thisptr).update()
    
    def startUpdateThread(self, milliseconds):
        deref(self.thisptr).startUpdateThread(milliseconds)

    def getCurrentPose(self):
         cdef string* data = new string()
         deref(self.wrapper).getCurrentPose(data)
         pyPose = types.Pose()
         pyPose.ParseFromString(deref(data))
         del data
         return pyPose

