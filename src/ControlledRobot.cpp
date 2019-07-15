#include "ControlledRobot.hpp"
#include <iostream>

using namespace std;
using namespace controlledRobot;


ControlledRobot::ControlledRobot(TransportSharedPtr commandTransport,TransportSharedPtr telemetryTransport):UpdateThread(),
    commandTransport(commandTransport),
    telemetryTransport(telemetryTransport)
{
}

void ControlledRobot::update()
{
    while (receiveRequest() != NO_DATA){};
}

ControlMessageType ControlledRobot::receiveRequest()
{
    std::string msg;
    int result = commandTransport->receive(&msg,controlledRobot::Transport::NOBLOCK);
    if (result){
        ControlMessageType reqestType = evaluateRequest(msg);
        return reqestType;
    }
    return NO_DATA;
}

ControlMessageType ControlledRobot::evaluateRequest(const std::string& request)
{

    uint16_t* type = (uint16_t*)request.data();
    ControlMessageType msgtype = (ControlMessageType)*type;
    std::string serializedMessage(request.data()+sizeof(uint16_t),request.size()-sizeof(uint16_t));

    switch (msgtype){
        case TARGET_POSE_COMMAND:{
            targetPose.lock();
            targetPose.get_ref().ParseFromString(serializedMessage);
            targetPose.unlock();
            commandTransport->send(serializeControlMessageType(TARGET_POSE_COMMAND));
            return TARGET_POSE_COMMAND;
        }
        case TWIST_COMMAND:{
            twistCommand.lock();
            twistCommand.get_ref().ParseFromString(serializedMessage);
            twistCommand.unlock();
            commandTransport->send(serializeControlMessageType(TWIST_COMMAND));
            return TWIST_COMMAND;
        }
        
        default:{
            commandTransport->send(serializeControlMessageType(NO_DATA));
            return msgtype;
        } 
    }

}


int ControlledRobot::setCurrentPose(const Pose& pose){
    return sendTelemetry(pose,CURRENT_POSE);
}

int ControlledRobot::setJointState(const JointState& state){
    return sendTelemetry(state,JOINT_STATE);
}


void ControlledRobot::addTelemetryMessageType(std::string &buf, const TelemetryMessageType& type){
    int currsize = buf.size();
    buf.resize(currsize + sizeof(uint16_t));
    uint16_t typeint = type;
    uint16_t* data = (uint16_t*)(buf.data()+currsize);
    *data = typeint;
}

void ControlledRobot::addControlMessageType(std::string &buf, const ControlMessageType& type){
    int currsize = buf.size();
    buf.resize(currsize + sizeof(uint16_t));
    uint16_t typeint = type;
    uint16_t* data = (uint16_t*)(buf.data()+currsize);
    *data = typeint;
}

std::string ControlledRobot::serializeControlMessageType(const ControlMessageType& type){
    std::string buf;
    addControlMessageType(buf,type);
    return buf;
}

std::string ControlledRobot::serializeCurrentPose()
{
    std::string buf;
    addTelemetryMessageType(buf,CURRENT_POSE);
    currentPose.get().AppendToString(&buf);
    return buf;

}