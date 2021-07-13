#include "ControlledRobot.hpp"
#include <iostream>


namespace robot_remote_control {


ControlledRobot::ControlledRobot(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport):UpdateThread(),
    commandTransport(commandTransport),
    telemetryTransport(telemetryTransport),
    heartbeatAllowedLatency(0.1),
    buffers(std::make_shared<TelemetryBuffer>()),
    logLevel(CUSTOM-1) {
    registerCommandType(TARGET_POSE_COMMAND, &poseCommand);
    registerCommandType(TWIST_COMMAND, &twistCommand);
    registerCommandType(GOTO_COMMAND, &goToCommand);
    registerCommandType(SIMPLE_ACTIONS_COMMAND, &simpleActionsCommand);
    registerCommandType(COMPLEX_ACTION_COMMAND, &complexActionCommandBuffer);
    registerCommandType(JOINTS_COMMAND, &jointsCommand);
    registerCommandType(HEARTBEAT, &heartbeatCommand);
    registerCommandType(PERMISSION, &permissionCommand);
    registerCommandType(ROBOT_TRAJECTORY_COMMAND, &robotTrajectoryCommand);


    registerTelemetryType<Pose>(CURRENT_POSE);
    registerTelemetryType<JointState>(JOINT_STATE);
    registerTelemetryType<JointState>(CONTROLLABLE_JOINTS);
    registerTelemetryType<SimpleActions>(SIMPLE_ACTIONS);
    registerTelemetryType<ComplexActions>(COMPLEX_ACTIONS);
    registerTelemetryType<RobotName>(ROBOT_NAME);
    registerTelemetryType<RobotState>(ROBOT_STATE);
    registerTelemetryType<LogMessage>(LOG_MESSAGE);
    registerTelemetryType<VideoStreams>(VIDEO_STREAMS);
    registerTelemetryType<SimpleSensors>(SIMPLE_SENSOR_DEFINITION);
    // simple sensors are stored in separate buffer when receiving, but sending requires this for requests
    registerTelemetryType<SimpleSensor>(SIMPLE_SENSOR_VALUE);
    registerTelemetryType<WrenchState>(WRENCH_STATE);
    registerTelemetryType<MapsDefinition>(MAPS_DEFINITION);
    registerTelemetryType<Map>(MAP); // TODO: needed? ()
    registerTelemetryType<Poses>(POSES);
    registerTelemetryType<Transforms>(TRANSFORMS);
    //registerTelemetryType<PermissionRequest>(PERMISSION_REQUEST); //no need to buffer, fills future
    registerTelemetryType<PointCloud>(POINTCLOUD);
    registerTelemetryType<IMU>(IMU_VALUES);
}

void ControlledRobot::update() {
    while (receiveRequest() != NO_CONTROL_DATA) {}

    if (heartbeatCommand.read(&heartbeatValues)) {
        connected.lockedAccess().set(true);
        // printf("received new HB params %.2f, %.2f\n", heartbeatValues.heartbeatduration(), heartbeatValues.heartbeatlatency());
        heartbeatTimer.start(heartbeatValues.heartbeatduration() + heartbeatAllowedLatency);
    }
    if (heartbeatTimer.isExpired()) {
        connected.lockedAccess().set(false);
        float elapsedTime = heartbeatTimer.getElapsedTime();
        if (heartbeatExpiredCallback != nullptr) {
            heartbeatExpiredCallback(elapsedTime);
        }
    }
}

ControlMessageType ControlledRobot::receiveRequest() {
    std::string msg;
    Transport::Flags flags = Transport::NONE;
    // if (!this->threaded()){
        flags = Transport::NOBLOCK;
    // }
    int result = commandTransport->receive(&msg, flags);
    if (result) {
        ControlMessageType requestType = evaluateRequest(msg);
        return requestType;
    }
    return NO_CONTROL_DATA;
}

ControlMessageType ControlledRobot::evaluateRequest(const std::string& request) {
    uint16_t* type = reinterpret_cast<uint16_t*>(const_cast<char*>(request.data()));
    ControlMessageType msgtype = (ControlMessageType)*type;
    std::string serializedMessage(request.data()+sizeof(uint16_t), request.size()-sizeof(uint16_t));

    switch (msgtype) {
        case TELEMETRY_REQUEST: {
            uint16_t* requestedtype = reinterpret_cast<uint16_t*>(const_cast<char*>(serializedMessage.data()));
            TelemetryMessageType type = (TelemetryMessageType) *requestedtype;
            std::string reply = buffers->peekSerialized(type);
            commandTransport->send(reply);
            return TELEMETRY_REQUEST;
        }
        case MAP_REQUEST: {
            uint16_t* requestedMap = reinterpret_cast<uint16_t*>(const_cast<char*>(serializedMessage.data()));
            std::string map;
            //get map
            {
                auto lockedAccess = mapBuffer.lockedAccess();
                if (*requestedMap < lockedAccess.get().size()){
                    RingBufferAccess::peekData(lockedAccess.get()[*requestedMap],&map);
                }
            }
            commandTransport->send(map);
            return MAP_REQUEST;
        }
        case LOG_LEVEL_SELECT: {
            logLevel = *reinterpret_cast<uint16_t*>(const_cast<char*>(serializedMessage.data()));
            commandTransport->send(serializeControlMessageType(LOG_LEVEL_SELECT));
            return LOG_LEVEL_SELECT;
        }
        case PERMISSION: {
            Permission perm;
            perm.ParseFromString(serializedMessage);
            std::promise<bool> &promise = pendingPermissionRequests[perm.requestuid()];
            try {
                promise.set_value(perm.granted());
            } catch (const std::future_error &e) {
                printf("%s\n", e.what());
            }
        }
        default: {
            CommandBufferBase * cmdbuffer = commandbuffers[msgtype];
            if (cmdbuffer) {
                if (!cmdbuffer->write(serializedMessage)) {
                    printf("unable to parse message of type %i in %s:%i\n", msgtype, __FILE__, __LINE__);
                    commandTransport->send(serializeControlMessageType(NO_CONTROL_DATA));
                    return NO_CONTROL_DATA;
                }
                commandTransport->send(serializeControlMessageType(msgtype));
                return msgtype;

            } else {
                commandTransport->send(serializeControlMessageType(NO_CONTROL_DATA));
                return msgtype;
            }
        }
    }
}


int ControlledRobot::setRobotState(const std::string& state) {
    RobotState protostate;
    protostate.set_state(state);
    return sendTelemetry(protostate, ROBOT_STATE);
}

int ControlledRobot::setLogMessage(enum LogLevel lvl, const std::string& message) {
    if (lvl <= logLevel || lvl >= CUSTOM) {
        LogMessage msg;
        msg.set_level(lvl);
        msg.set_message(message);
        return sendTelemetry(msg, LOG_MESSAGE);
    }
    return -1;
}

int ControlledRobot::setLogMessage(const LogMessage& log_message) {
    if (log_message.level() <= logLevel || log_message.level() >= CUSTOM) {
        return sendTelemetry(log_message, LOG_MESSAGE);
    }
    return -1;
}

robot_remote_control::TimeStamp ControlledRobot::getTime() {
    struct timespec rawtime;
    clock_gettime(CLOCK_REALTIME, &rawtime);
    robot_remote_control::TimeStamp timestamp;
    timestamp.set_secs(rawtime.tv_sec);
    timestamp.set_nsecs(rawtime.tv_nsec);
    return timestamp;
}

void ControlledRobot::addTelemetryMessageType(std::string *buf, const TelemetryMessageType& type) {
    int currsize = buf->size();
    buf->resize(currsize + sizeof(uint16_t));
    uint16_t* data = reinterpret_cast<uint16_t*>(const_cast<char*>(buf->data()+currsize));
    *data = type;
}

void ControlledRobot::addControlMessageType(std::string *buf, const ControlMessageType& type) {
    int currsize = buf->size();
    buf->resize(currsize + sizeof(uint16_t));
    uint16_t* data = reinterpret_cast<uint16_t*>(const_cast<char*>(buf->data()+currsize));
    *data = type;
}

std::string ControlledRobot::serializeControlMessageType(const ControlMessageType& type) {
    std::string buf;
    addControlMessageType(&buf, type);
    return buf;
}


}  // namespace robot_remote_control
