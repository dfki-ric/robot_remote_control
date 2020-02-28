#include "RobotController.hpp"
#include <iostream>


using namespace std;
using namespace robot_remote_control;


RobotController::RobotController(TransportSharedPtr commandTransport,TransportSharedPtr telemetryTransport, size_t recv_buffer_size):UpdateThread(),
    commandTransport(commandTransport),
    telemetryTransport(telemetryTransport)
{
    buffers = std::shared_ptr<TelemetryBuffer>(new TelemetryBuffer(recv_buffer_size));
    simplesensorbuffer = std::shared_ptr<SimpleSensorBuffer>(new SimpleSensorBuffer());
}

RobotController::~RobotController(){

}

void RobotController::setTargetPose(const Pose & pose)
{
    sendProtobufData(pose,TARGET_POSE_COMMAND);
}

void RobotController::setLeftArmEndEffectorTargetPose(const Pose & pose)
{
    sendProtobufData(pose, LEFT_ARM_END_EFFECTOR_TARGET_POSE_COMMAND);
}

void RobotController::setTwistCommand(const Twist &twistCommand)
{
    sendProtobufData(twistCommand,TWIST_COMMAND);
}

void RobotController::setGoToCommand(const GoTo &goToCommand) {
    sendProtobufData(goToCommand, GOTO_COMMAND);
}

void RobotController::setJointCommand(const JointState &jointsCommand){
    sendProtobufData(jointsCommand, JOINTS_COMMAND);
}

void RobotController::setSimpleActionCommand(const SimpleAction &simpleActionCommand) {
    SimpleActions action;
    (*action.add_actions()) = simpleActionCommand;
    sendProtobufData(action, SIMPLE_ACTIONS_COMMAND);
}

void RobotController::setComplexActionCommand(const ComplexAction &complexActionCommand) {
    ComplexActions action;
     (*action.add_actions()) = complexActionCommand;
    sendProtobufData(action, COMPLEX_ACTIONS_COMMAND);
}

void RobotController::setLogLevel(const uint32_t &level){
    std::string buf;
    buf.resize(sizeof(uint16_t) + sizeof(uint32_t));
    uint16_t* data = reinterpret_cast<uint16_t*>(const_cast<char*>(buf.data()));
    *data = LOG_LEVEL_SELECT;

    uint32_t *levelptr = reinterpret_cast<uint32_t*>(const_cast<char*>(buf.data()+sizeof(uint16_t)));
    *levelptr = level;
    sendRequest(buf);
}

void RobotController::update() {
    if (telemetryTransport.get()) {
        std::string buf;
        Transport::Flags flags = Transport::NONE;
        // if (!this->threaded()){
            flags = Transport::NOBLOCK;
        // }
        while (telemetryTransport->receive(&buf, flags)) {
            evaluateTelemetry(buf);
        }
    } else {
        printf("ERROR no telemetry Transport set\n");
    }
}


std::string RobotController::sendRequest(const std::string& serializedMessage) {
    commandTransport->send(serializedMessage);
    std::string replystr;

    // blocking receive
    while (commandTransport->receive(&replystr) == 0) {
        // wait time depends on how long the transports recv blocks
    }

    return replystr;
}

TelemetryMessageType RobotController::evaluateTelemetry(const std::string& reply) {
    uint16_t* type = reinterpret_cast<uint16_t*>(const_cast<char*>(reply.data()));

    std::string serializedMessage(reply.data()+sizeof(uint16_t), reply.size()-sizeof(uint16_t));

    TelemetryMessageType msgtype = (TelemetryMessageType)*type;

    switch (msgtype) {
        case CURRENT_POSE:              addToTelemetryBuffer< Pose              >(msgtype, serializedMessage);
                                        return msgtype;
        case JOINT_STATE:               addToTelemetryBuffer< JointState        >(msgtype, serializedMessage);
                                        return msgtype;
        case CONTROLLABLE_JOINTS:       addToTelemetryBuffer< JointState        >(msgtype, serializedMessage);
                                        return msgtype;
        case SIMPLE_ACTIONS:            addToTelemetryBuffer< SimpleActions     >(msgtype, serializedMessage);
                                        return msgtype;
        case COMPLEX_ACTIONS:           addToTelemetryBuffer< ComplexActions    >(msgtype, serializedMessage);
                                        return msgtype;
        case ROBOT_NAME:                addToTelemetryBuffer< RobotName         >(msgtype, serializedMessage);
                                        return msgtype;
        case ROBOT_STATE:               addToTelemetryBuffer< RobotState        >(msgtype, serializedMessage);
                                        return msgtype;
        case LOG_MESSAGE:               addToTelemetryBuffer< LogMessage        >(msgtype, serializedMessage);
                                        return msgtype;
        case VIDEO_STREAMS:             addToTelemetryBuffer< VideoStreams      >(msgtype, serializedMessage);
                                        return msgtype;
        case SIMPLE_SENSOR_DEFINITION:  addToTelemetryBuffer< SimpleSensors     >(msgtype, serializedMessage);
                                        return msgtype;

        // multi values in single stream
        case SIMPLE_SENSOR_VALUE:       addToSimpleSensorBuffer(serializedMessage);
                                        return msgtype;

        case TELEMETRY_MESSAGE_TYPES_NUMBER:
        case NO_TELEMETRY_DATA:
        {
            return msgtype;
        }
    }

    // should never reach this
    return NO_TELEMETRY_DATA;
}

void RobotController::addToSimpleSensorBuffer(const std::string &serializedMessage) {
    SimpleSensor data;
    data.ParseFromString(serializedMessage);
    // check if buffer number is big enough
    // size must be id+1 (id 0 needs size 1)
    simplesensorbuffer->initBufferID(data.id());
    // if (simplesensorbuffer->size() <= data.id()){
    //     simplesensorbuffer->resize(data.id());
    // }

    simplesensorbuffer->lock();
    RingBufferAccess::pushData(simplesensorbuffer->get_ref()[data.id()], data, true);
    simplesensorbuffer->unlock();

}