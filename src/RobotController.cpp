#include "RobotController.hpp"
#include <iostream>

using namespace std;
using namespace interaction;


RobotController::RobotController(TransportSharedPtr commandTransport,TransportSharedPtr telemetryTransport):
    commandTransport(commandTransport),
    telemetryTransport(telemetryTransport)
{
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


void RobotController::updateTelemetry(){
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
            currentPose.ParseFromString(serializedMessage);
            return msgtype;
        } 
        
        default: return msgtype;
    }

    //should never reach this
    return NO_DATA;
}