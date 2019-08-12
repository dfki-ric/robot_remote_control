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

                                    // CURRENT_POSE,       //the curretn Pose of the robot base
                                    // JOINT_STATE,         //current Joint values
                                    // JOINT_NAME_REPLY,   // the names of the controllable joints
                                    // SIMPLE_ACTIONS_NAMES_REPLY, // the names of the simple actions
                                    // COMPLEX_ACTIONS_NAMES_REPLY, // the names of the complex actions

        unlock();
    }

    TelemetryBuffer::~TelemetryBuffer(){

    }




