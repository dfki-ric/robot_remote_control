#include "RobotController.hpp"
#include <iostream>

using namespace std;
using namespace interaction;


RobotController::RobotController(TransportSharedPtr commandTransport,TransportSharedPtr telemetryTransport):UpdateThread(),
    commandTransport(commandTransport),
    telemetryTransport(telemetryTransport)
{
    initBuffers(10);
}

RobotController::~RobotController(){

}


void RobotController::setTargetPose(const interaction::Pose & pose)
{
    sendProtobufData(pose,TARGET_POSE_COMMAND);
}


void RobotController::setTwistCommand(const interaction::Twist &twistCommand)
{
    sendProtobufData(twistCommand,TWIST_COMMAND);
}


void RobotController::update(){
    if (telemetryTransport.get()){
        std::string buf;
        while(telemetryTransport->receive(&buf,interaction::Transport::NOBLOCK)){
            evaluateReply(buf);
        }
    }
}


std::string RobotController::sendRequest(const std::string& serializedMessage){
    
    commandTransport->send(serializedMessage);
    std::string replystr;

    //blocking receive
    commandTransport->receive(&replystr);

    return replystr;
}

ControlMessageType RobotController::evaluateReply(const std::string& reply){


    uint16_t* type = (uint16_t*)reply.data();

    std::string serializedMessage(reply.data()+sizeof(uint16_t),reply.size()-sizeof(uint16_t));

    ControlMessageType msgtype = (ControlMessageType)*type;

    switch (msgtype){
        case CURRENT_POSE:{
            interaction::Pose currentPose;
            currentPose.ParseFromString(serializedMessage);
            RingBufferAccess::pushData(buffers.get()[CURRENT_POSE],currentPose);
            return msgtype;
        }
        case JOINT_STATE:{
            interaction::JointState currentJointState;
            currentJointState.ParseFromString(serializedMessage);
            RingBufferAccess::pushData(buffers.get()[JOINT_STATE],currentJointState);
            return msgtype;
        }
        
        default: return msgtype;
    }

    //should never reach this
    return NO_DATA;
}


void RobotController::initBuffers(const unsigned int &defaultSize){
    //create vector of shared ptr
    buffers.get().resize(TELEMETRY_MESSAGE_TYPES_NUMBER);

    std::shared_ptr<RingBufferBase> newbuf;

    //fill shared pointers with objects
    newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<interaction::Pose>(defaultSize));
    buffers.get()[CURRENT_POSE] = newbuf;

    newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<interaction::JointState>(defaultSize));
    buffers.get()[JOINT_STATE] = newbuf;
}