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
        get_ref()[JOINT_NAME_REPLY] = newbuf;

        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<controlledRobot::SimpleActions>(size));
        get_ref()[SIMPLE_ACTIONS_NAMES_REPLY] = newbuf;

        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<controlledRobot::ComplexActions>(size));
        get_ref()[COMPLEX_ACTIONS_NAMES_REPLY] = newbuf;

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
            case JOINT_NAME_REPLY:{
                controlledRobot::JointState data;
                fillBuffer(JOINT_NAME_REPLY,data,buf);
                break;
            }
            case SIMPLE_ACTIONS_NAMES_REPLY:{
                controlledRobot::SimpleActions data;
                fillBuffer(SIMPLE_ACTIONS_NAMES_REPLY,data,buf);
                break;
            }
            case COMPLEX_ACTIONS_NAMES_REPLY:{
                controlledRobot::ComplexActions data;
                fillBuffer(COMPLEX_ACTIONS_NAMES_REPLY,data,buf);
                break;
            }
            case NO_TELEMETRY_DATA:
            case TELEMETRY_MESSAGE_TYPES_NUMBER:
            break;

        }

        
        unlock();
        return buf;
    }

