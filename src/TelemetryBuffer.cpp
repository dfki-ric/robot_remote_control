#include "TelemetryBuffer.hpp"
#include "MessageTypes.hpp"

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




