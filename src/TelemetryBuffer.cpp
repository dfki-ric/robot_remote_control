#include "TelemetryBuffer.hpp"


using namespace robot_remote_control;


    TelemetryBuffer::TelemetryBuffer(const size_t &size){
        //create vector of shared ptr
        lock();
        get_ref().resize(TELEMETRY_MESSAGE_TYPES_NUMBER);

        std::shared_ptr<RingBufferBase> newbuf;

        //fill shared pointers with objects
        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<Pose>(size));
        get_ref()[CURRENT_POSE] = newbuf;

        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<JointState>(size));
        get_ref()[JOINT_STATE] = newbuf;

        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<JointState>(size));
        get_ref()[CONTROLLABLE_JOINTS] = newbuf;

        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<SimpleActions>(size));
        get_ref()[SIMPLE_ACTIONS] = newbuf;

        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<ComplexActions>(size));
        get_ref()[COMPLEX_ACTIONS] = newbuf;

        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<RobotName>(size));
        get_ref()[ROBOT_NAME] = newbuf;

        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<RobotState>(size));
        get_ref()[ROBOT_STATE] = newbuf;

        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<LogMessage>(size));
        get_ref()[LOG_MESSAGE] = newbuf;

        unlock();
    }

    TelemetryBuffer::~TelemetryBuffer(){

    }


    std::string TelemetryBuffer::peekSerialized(const TelemetryMessageType &type){
        
        std::string buf("");
        lock();
        
        switch (type){
            case CURRENT_POSE:{
                Pose pose;
                fillBuffer(CURRENT_POSE,pose,buf);
                break;
            }
            case JOINT_STATE:{
                JointState data;
                fillBuffer(JOINT_STATE,data,buf);
                break;
            }
            case CONTROLLABLE_JOINTS:{
                JointState data;
                fillBuffer(CONTROLLABLE_JOINTS,data,buf);
                break;
            }
            case SIMPLE_ACTIONS:{
                SimpleActions data;
                fillBuffer(SIMPLE_ACTIONS,data,buf);
                break;
            }
            case COMPLEX_ACTIONS:{
                ComplexActions data;
                fillBuffer(COMPLEX_ACTIONS,data,buf);
                break;
            }
            case ROBOT_NAME:{
                RobotName data;
                fillBuffer(ROBOT_NAME,data,buf);
                break;
            }
            case ROBOT_STATE:{
                RobotState data;
                fillBuffer(ROBOT_STATE,data,buf);
                break;
            }
            case LOG_MESSAGE:{
                LogMessage data;
                fillBuffer(LOG_MESSAGE,data,buf);
                break;
            }
            case NO_TELEMETRY_DATA:
            case TELEMETRY_MESSAGE_TYPES_NUMBER:
            break;

        }

        
        unlock();
        return buf;
    }

