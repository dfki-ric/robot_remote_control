#include "RobotController.hpp"
#include <iostream>

using namespace std;
using namespace controlledRobot;


RobotController::RobotController(TransportSharedPtr commandTransport,TransportSharedPtr telemetryTransport):UpdateThread(),
    commandTransport(commandTransport),
    telemetryTransport(telemetryTransport)
{
    std::shared_ptr<TelemetryBuffer> buffers = std::shared_ptr<TelemetryBuffer>(new TelemetryBuffer(10));
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

void RobotController::setSimpleActionsCommand(const SimpleActions &simpleActionsCommand) {
    sendProtobufData(simpleActionsCommand, SIMPLE_ACTIONS_COMMAND);
}

 void RobotController::setComplexActionsCommand(const ComplexActions &complexActionsCommand) {
     sendProtobufData(complexActionsCommand, COMPLEX_ACTIONS_COMMAND);
 }

void RobotController::update(){
    if (telemetryTransport.get()){
        std::string buf;
        controlledRobot::Transport::Flags flags = Transport::NONE;
        //if (!this->threaded()){
            flags = Transport::NOBLOCK;
        //}
        while(telemetryTransport->receive(&buf,flags)){
            evaluateTelemetry(buf);
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

TelemetryMessageType RobotController::evaluateTelemetry(const std::string& reply){

    uint16_t* type = (uint16_t*)reply.data();

    std::string serializedMessage(reply.data()+sizeof(uint16_t),reply.size()-sizeof(uint16_t));

    TelemetryMessageType msgtype = (TelemetryMessageType)*type;

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
        case CONTROLLABLE_JOINTS: {
            JointState controllableJoints;
            controllableJoints.ParseFromString(serializedMessage);
            buffers.lock();
            RingBufferAccess::pushData(buffers.get_ref()[CONTROLLABLE_JOINTS],controllableJoints);
            buffers.unlock();
            return msgtype;
        }
        case SIMPLE_ACTIONS: {
            SimpleActions simpleActions;
            simpleActions.ParseFromString(serializedMessage);
            buffers.lock();
            RingBufferAccess::pushData(buffers.get_ref()[SIMPLE_ACTIONS],simpleActions);
            buffers.unlock();
            return msgtype;
        }
        case COMPLEX_ACTIONS: {
            ComplexActions complexActions;
            complexActions.ParseFromString(serializedMessage);
            buffers.lock();
            RingBufferAccess::pushData(buffers.get_ref()[COMPLEX_ACTIONS],complexActions);
            buffers.unlock();
            return msgtype;
        }
        case ROBOT_NAME: {
            RobotName robotName;
            robotName.ParseFromString(serializedMessage);
            buffers.lock();
            RingBufferAccess::pushData(buffers.get_ref()[ROBOT_NAME],robotName);
            buffers.unlock();
            return msgtype;
        }
        
        default: return msgtype;
    }

    //should never reach this
    return NO_TELEMETRY_DATA;
}
