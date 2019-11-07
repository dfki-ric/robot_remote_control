#include "ControlledRobot.hpp"
#include <iostream>

using namespace std;
using namespace robot_remote_control;


ControlledRobot::ControlledRobot(TransportSharedPtr commandTransport,TransportSharedPtr telemetryTransport):UpdateThread(),
    commandTransport(commandTransport),
    telemetryTransport(telemetryTransport),
    logLevel(CUSTOM-1)
{
}

void ControlledRobot::update()
{
    while (receiveRequest() != NO_CONTROL_DATA){};
}

ControlMessageType ControlledRobot::receiveRequest()
{
    std::string msg;
    Transport::Flags flags = Transport::NONE;
    //if (!this->threaded()){
        flags = Transport::NOBLOCK;
    //}
    int result = commandTransport->receive(&msg,flags);
    if (result){
        ControlMessageType reqestType = evaluateRequest(msg);
        return reqestType;
    }
    return NO_CONTROL_DATA;
}

ControlMessageType ControlledRobot::evaluateRequest(const std::string& request)
{

    uint16_t* type = (uint16_t*)request.data();
    ControlMessageType msgtype = (ControlMessageType)*type;
    std::string serializedMessage(request.data()+sizeof(uint16_t),request.size()-sizeof(uint16_t));

    switch (msgtype){
        case TARGET_POSE_COMMAND:{
            poseCommand.write(serializedMessage);
            commandTransport->send(serializeControlMessageType(TARGET_POSE_COMMAND));
            return TARGET_POSE_COMMAND;
        }
        case TWIST_COMMAND:{
            twistCommand.write(serializedMessage);
            commandTransport->send(serializeControlMessageType(TWIST_COMMAND));
            return TWIST_COMMAND;
        }
        case JOINTS_COMMAND: {
            jointsCommand.write(serializedMessage);
            commandTransport->send(serializeControlMessageType(JOINTS_COMMAND));
            return JOINTS_COMMAND;
        }
        case GOTO_COMMAND: {
            goToCommand.write(serializedMessage);
            commandTransport->send(serializeControlMessageType(GOTO_COMMAND));
            return GOTO_COMMAND;
        }
        case SIMPLE_ACTIONS_COMMAND: {
            simpleActionsCommand.write(serializedMessage);
            commandTransport->send(serializeControlMessageType(SIMPLE_ACTIONS_COMMAND));
            return SIMPLE_ACTIONS_COMMAND;
        }
        case COMPLEX_ACTIONS_COMMAND: {
            complexActionsCommand.write(serializedMessage);
            commandTransport->send(serializeControlMessageType(COMPLEX_ACTIONS_COMMAND));
            return COMPLEX_ACTIONS_COMMAND;
        }
        case TELEMETRY_REQUEST:{
            uint16_t* requestedtype = (uint16_t*)(serializedMessage.data());
            TelemetryMessageType type = (TelemetryMessageType) *requestedtype;
            std::string reply = buffers.peekSerialized(type);
            commandTransport->send(reply);
            return TELEMETRY_REQUEST;
        }
        case LOG_LEVEL_SELECT:{
            logLevel = *(uint32_t*)(serializedMessage.data());
            commandTransport->send(serializeControlMessageType(LOG_LEVEL_SELECT));
            return LOG_LEVEL_SELECT;
        }
        
        default:{
            commandTransport->send(serializeControlMessageType(NO_CONTROL_DATA));
            return msgtype;
        } 
    }

}


int ControlledRobot::setRobotState(const std::string& state){
    RobotState protostate;
    protostate.set_state(state);
    return sendTelemetry(protostate, ROBOT_STATE);

}

int ControlledRobot::setLogMessage(enum LogLevel lvl, const std::string& message){
    if (lvl <= logLevel || lvl >= CUSTOM){
        LogMessage msg;
        msg.set_level(lvl);
        msg.set_message(message);
        return sendTelemetry(msg, LOG_MESSAGE);
    }   
    return -1;
}

int ControlledRobot::setLogMessage(const LogMessage& log_message){
    if (log_message.level() <= logLevel || log_message.level() >= CUSTOM){
        return sendTelemetry(log_message, LOG_MESSAGE);
    }
    return -1;
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

// std::string ControlledRobot::serializeCurrentPose()
// {
//     std::string buf;
//     addTelemetryMessageType(buf,CURRENT_POSE);
//     currentPose.get().AppendToString(&buf);
//     return buf;

// }
