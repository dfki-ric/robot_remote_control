#include "ControlledRobot.hpp"
#include <iostream>

using namespace std;
using namespace robot_remote_control;


ControlledRobot::ControlledRobot(TransportSharedPtr commandTransport,TransportSharedPtr telemetryTransport):UpdateThread(),
    commandTransport(commandTransport),
    telemetryTransport(telemetryTransport),
    logLevel(CUSTOM-1)
{
    twistCommandGetCounter.set(0);
    jointsCommandGetCounter.set(-1);
    goToCommandGetCounter.set(-1);
    simpleActionsCommandGetCounter.set(0);
    complexActionsCommandGetCounter.set(0);
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
            twistCommandGetCounter.set(0);
            return TWIST_COMMAND;
        }
        case JOINTS_COMMAND: {
            jointsCommand.lock();
            jointsCommand.get_ref().ParseFromString(serializedMessage);
            jointsCommand.unlock();
            commandTransport->send(serializeControlMessageType(JOINTS_COMMAND));
            jointsCommandGetCounter.set(0);
            return JOINTS_COMMAND;
        }
        case GOTO_COMMAND: {
            goToCommand.lock();
            goToCommand.get_ref().ParseFromString(serializedMessage);
            goToCommand.unlock();
            commandTransport->send(serializeControlMessageType(GOTO_COMMAND));
            goToCommandGetCounter.set(0);
            return GOTO_COMMAND;
        }
        case SIMPLE_ACTIONS_COMMAND: {
            simpleActionsCommand.lock();
            simpleActionsCommand.get_ref().ParseFromString(serializedMessage);
            simpleActionsCommand.unlock();
            commandTransport->send(serializeControlMessageType(SIMPLE_ACTIONS_COMMAND));
            simpleActionsCommandGetCounter.set(0);
            return SIMPLE_ACTIONS_COMMAND;
        }
        case COMPLEX_ACTIONS_COMMAND: {
            complexActionsCommand.lock();
            complexActionsCommand.get_ref().ParseFromString(serializedMessage);
            complexActionsCommand.unlock();
            commandTransport->send(serializeControlMessageType(COMPLEX_ACTIONS_COMMAND));
            complexActionsCommandGetCounter.set(0);
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


int ControlledRobot::setCurrentPose(const Pose& pose){
    return sendTelemetry(pose,CURRENT_POSE);
}

int ControlledRobot::setJointState(const JointState& state){
    return sendTelemetry(state,JOINT_STATE);
}

int ControlledRobot::setControllableJoints(const JointState& controllableJoints) {
    return sendTelemetry(controllableJoints, CONTROLLABLE_JOINTS);
}

int ControlledRobot::setSimpleActions(const SimpleActions& simpleActions) {
    return sendTelemetry(simpleActions, SIMPLE_ACTIONS);
}

int ControlledRobot::setComplexActions(const ComplexActions& complexActions) {
    return sendTelemetry(complexActions, COMPLEX_ACTIONS);
}

int ControlledRobot::setRobotName(const RobotName& robotName) {
    return sendTelemetry(robotName, ROBOT_NAME);
}

int ControlledRobot::setRobotState(const std::string& state){
    RobotState protostate;
    protostate.set_state(state);
    return sendTelemetry(protostate, ROBOT_STATE);

}

int ControlledRobot::setLogMessage(enum LogLevel lvl, const std::string& message){
    if (lvl <= logLevel){
        LogMessage msg;
        msg.set_type(lvl);
        msg.set_message(message);
        return sendTelemetry(msg, LOG_MESSAGE);
    } else return -1;
}

int ControlledRobot::setLogMessage(const LogMessage& log_message){
    if (log_message.type() <= logLevel){
        return sendTelemetry(log_message, LOG_MESSAGE);
    } else return -1;
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