#include "RobotController.hpp"
#include <iostream>

using namespace std;
using namespace controlledRobot;


RobotController::RobotController(TransportSharedPtr commandTransport,TransportSharedPtr telemetryTransport):UpdateThread(),
    commandTransport(commandTransport),
    telemetryTransport(telemetryTransport)
{
    initBuffers(10);
}

RobotController::~RobotController(){

}


void RobotController::setTargetPose(const Pose & pose)
{
    sendProtobufData(pose,TARGET_POSE_COMMAND);
}


void RobotController::setTwistCommand(const Twist &twistCommand)
{
    sendProtobufData(twistCommand,TWIST_COMMAND);
}

void RobotController::setGoToCommand(const GoTo &goToCommand) {
    sendProtobufData(goToCommand, GOTO_COMMAND);
}

void RobotController::update(){
    if (telemetryTransport.get()){
        std::string buf;
        while(telemetryTransport->receive(&buf,Transport::NOBLOCK)){
            evaluateReply(buf);
        }
    }else{
        printf("ERROR no telemetry Transport set\n");
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
            Pose currentPose;
            currentPose.ParseFromString(serializedMessage);
            buffers.lock();
            RingBufferAccess::pushData(buffers.get_ref()[CURRENT_POSE],currentPose);
            buffers.unlock();
            return msgtype;
        }
        case JOINT_STATE:{
            JointState currentJointState;
            currentJointState.ParseFromString(serializedMessage);
            buffers.lock();
            RingBufferAccess::pushData(buffers.get_ref()[JOINT_STATE],currentJointState);
            buffers.unlock();
            return msgtype;
        }
        
        default: return msgtype;
    }

    //should never reach this
    return NO_DATA;
}


void RobotController::initBuffers(const unsigned int &defaultSize){
    //create vector of shared ptr
    buffers.lock();
    buffers.get_ref().resize(TELEMETRY_MESSAGE_TYPES_NUMBER);

    std::shared_ptr<RingBufferBase> newbuf;

    //fill shared pointers with objects
    newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<controlledRobot::Pose>(defaultSize));
    buffers.get_ref()[CURRENT_POSE] = newbuf;

    newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<controlledRobot::JointState>(defaultSize));
    buffers.get_ref()[JOINT_STATE] = newbuf;


    buffers.unlock();
}