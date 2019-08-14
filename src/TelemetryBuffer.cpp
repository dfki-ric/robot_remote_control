#include "TelemetryBuffer.hpp"


using namespace controlledRobot;


    TelemetryBuffer::TelemetryBuffer(const size_t &size){
        //create vector of shared ptr
        lock();
        get_ref().resize(TELEMETRY_MESSAGE_TYPES_NUMBER);

        std::shared_ptr<RingBufferBase> newbuf;

        //fill shared pointers with objects
        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<controlledRobot::Pose>(size));
        get_ref()[CURRENT_POSE] = newbuf;

        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<controlledRobot::JointState>(size));
        get_ref()[JOINT_STATE] = newbuf;

        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<controlledRobot::JointState>(size));
        get_ref()[CONTROLLABLE_JOINTS] = newbuf;

        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<controlledRobot::SimpleActions>(size));
        get_ref()[SIMPLE_ACTIONS] = newbuf;

        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<controlledRobot::ComplexActions>(size));
        get_ref()[COMPLEX_ACTIONS] = newbuf;

        unlock();
    }

    TelemetryBuffer::~TelemetryBuffer(){

    }


    std::string TelemetryBuffer::peekSerialized(const TelemetryMessageType &type){
        
        std::string buf("");
        lock();
        
        switch (type){
            case CURRENT_POSE:{
                controlledRobot::Pose pose;
                fillBuffer(CURRENT_POSE,pose,buf);
                break;
            }
            case JOINT_STATE:{
                controlledRobot::JointState data;
                fillBuffer(JOINT_STATE,data,buf);
                break;
            }
            case CONTROLLABLE_JOINTS:{
                controlledRobot::JointState data;
                fillBuffer(CONTROLLABLE_JOINTS,data,buf);
                break;
            }
            case SIMPLE_ACTIONS:{
                controlledRobot::SimpleActions data;
                fillBuffer(SIMPLE_ACTIONS,data,buf);
                break;
            }
            case COMPLEX_ACTIONS:{
                controlledRobot::ComplexActions data;
                fillBuffer(COMPLEX_ACTIONS,data,buf);
                break;
            }
            case NO_TELEMETRY_DATA:
            case TELEMETRY_MESSAGE_TYPES_NUMBER:
            break;

        }

        
        unlock();
        return buf;
    }

