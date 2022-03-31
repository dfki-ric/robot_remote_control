#include "RobotController.hpp"
#include <iostream>
#include <memory>
#include <unistd.h>
#include <stdexcept>
#include <google/protobuf/io/coded_stream.h>

using namespace robot_remote_control;


RobotController::RobotController(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport, const size_t &buffersize, const float &maxLatency):UpdateThread(),
    commandTransport(commandTransport),
    telemetryTransport(telemetryTransport),
    heartBeatDuration(0),
    heartBreatRoundTripTime(0),
    maxLatency(maxLatency),
    buffers(std::make_shared<TelemetryBuffer>()),
    simplesensorbuffer(std::make_shared< SimpleBuffer<SimpleSensor> >()),
    connected(false) {
        registerTelemetryType<Pose>(CURRENT_POSE, buffersize);
        registerTelemetryType<JointState>(JOINT_STATE, buffersize);
        registerTelemetryType<JointState>(CONTROLLABLE_JOINTS, buffersize);
        registerTelemetryType<SimpleActions>(SIMPLE_ACTIONS, 1);  // this is a configuration, so no bigger buffer needed
        registerTelemetryType<ComplexActions>(COMPLEX_ACTIONS, 1);  // this is a configuration, so no bigger buffer needed
        registerTelemetryType<RobotName>(ROBOT_NAME, 1);  // this is a configuration, so no bigger buffer needed
        registerTelemetryType<RobotState>(ROBOT_STATE, buffersize);
        registerTelemetryType<LogMessage>(LOG_MESSAGE, buffersize);
        registerTelemetryType<VideoStreams>(VIDEO_STREAMS, 1);  // this is a configuration, so no bigger buffer needed
        registerTelemetryType<SimpleSensors>(SIMPLE_SENSOR_DEFINITION, 1);  // this is a configuration, so no bigger buffer needed
        registerTelemetryType<WrenchState>(WRENCH_STATE, buffersize);
        registerTelemetryType<Poses>(POSES, buffersize);
        registerTelemetryType<Transforms>(TRANSFORMS, buffersize);
        registerTelemetryType<PermissionRequest>(PERMISSION_REQUEST, buffersize);
        registerTelemetryType<PointCloud>(POINTCLOUD, buffersize);
        registerTelemetryType<IMU>(IMU_VALUES, buffersize);
        registerTelemetryType<ContactPoints>(CONTACT_POINTS, buffersize);
        registerTelemetryType<Twist>(CURRENT_TWIST, buffersize);
        registerTelemetryType<Acceleration>(CURRENT_ACCELERATION, buffersize);
        registerTelemetryType<CameraInformation>(CAMERA_INFORMATION, 1);  // this is a configuration, so no bigger buffer needed
        registerTelemetryType<Image>(IMAGE, buffersize);
        registerTelemetryType<ImageLayers>(IMAGE_LAYERS, buffersize);
        registerTelemetryType<Odometry>(ODOMETRY, buffersize);
        registerTelemetryType<ControllableFrames>(CONTROLLABLE_FRAMES, 1);  // this is a configuration, so no bigger buffer needed

        #ifdef RRC_STATISTICS
            // add names to buffer, this types have aspecial treatment, the should not be registered
            SimpleSensor simpleSensor;
            statistics.names[SIMPLE_SENSOR_VALUE] = simpleSensor.GetTypeName();
            MapsDefinition mapsDefinition;
            statistics.names[MAPS_DEFINITION] = mapsDefinition.GetTypeName();
            Map map;
            statistics.names[MAP] = map.GetTypeName();
        #endif

        lostConnectionCallback = [&](const float& time){
            printf("lost connection to robot, no reply for %f seconds\n", time);
        };
}

RobotController::~RobotController() {
    stopUpdateThread();
}

void RobotController::setTargetPose(const Pose & pose) {
    sendProtobufData(pose, TARGET_POSE_COMMAND);
}

void RobotController::setTwistCommand(const Twist &twistCommand) {
    sendProtobufData(twistCommand, TWIST_COMMAND);
}

void RobotController::setGoToCommand(const GoTo &goToCommand) {
    sendProtobufData(goToCommand, GOTO_COMMAND);
}

void RobotController::setJointCommand(const JointCommand &jointsCommand) {
    sendProtobufData(jointsCommand, JOINTS_COMMAND);
}

void RobotController::setSimpleActionCommand(const SimpleAction &simpleActionCommand) {
    sendProtobufData(simpleActionCommand, SIMPLE_ACTIONS_COMMAND);
}

void RobotController::setComplexActionCommand(const ComplexAction &complexActionCommand) {
    sendProtobufData(complexActionCommand, COMPLEX_ACTION_COMMAND);
}

void RobotController::setRobotTrajectoryCommand(const Poses &robotTrajectoryCommand) {
    sendProtobufData(robotTrajectoryCommand, ROBOT_TRAJECTORY_COMMAND);
}

void RobotController::setLogLevel(const uint16_t &level) {
    std::string buf;
    buf.resize(sizeof(uint16_t) + sizeof(uint16_t));
    uint16_t* data = reinterpret_cast<uint16_t*>(const_cast<char*>(buf.data()));
    *data = LOG_LEVEL_SELECT;

    uint16_t *levelptr = reinterpret_cast<uint16_t*>(const_cast<char*>(buf.data()+sizeof(uint16_t)));
    *levelptr = level;
    sendRequest(buf);
}

bool RobotController::setPermission(const Permission& permission) {
    sendProtobufData(permission, PERMISSION);
}

void RobotController::update() {
    if (telemetryTransport.get()) {
        std::string buf;
        Transport::Flags flags = Transport::NONE;
        // if (!this->threaded()){
            flags = Transport::NOBLOCK;
        // }
        while (telemetryTransport->receive(&buf, flags)) {
            uint16_t type = evaluateTelemetry(buf);
            if (type != NO_TELEMETRY_DATA) {
                auto callCb = [&](const std::function<void(const uint16_t & type)> &cb){cb(type);};
                std::for_each(telemetryReceivedCallbacks.begin(), telemetryReceivedCallbacks.end(), callCb);
            }
        }
    } else {
        printf("ERROR no telemetry Transport set\n");
    }

    if (heartBeatDuration != 0) {
        if (heartBeatTimer.isExpired()) {
            // TODO: check if send needed?
            if (commandTransport.get()) {
                HeartBeat hb;
                hb.set_heartbeatduration(heartBeatDuration);
                latencyTimer.start();
                std::string rep = sendProtobufData(hb, HEARTBEAT);
                float time = latencyTimer.getElapsedTime();
                heartBreatRoundTripTime.store(time);
                connected.store(true);
            }
            heartBeatTimer.start(heartBeatDuration);
        } else {
            heartBreatRoundTripTime.store(-1);
            connected.store(false);
        }
    }
}

bool RobotController::requestMap(Map *map, const uint16_t &mapId){
    std::string replybuf;
    bool result = requestBinary(mapId, &replybuf, MAP_REQUEST);
    google::protobuf::io::CodedInputStream cistream(reinterpret_cast<const uint8_t *>(replybuf.data()), replybuf.size());
    cistream.SetTotalBytesLimit(replybuf.size(), replybuf.size());
    map->ParseFromCodedStream(&cistream);
    return result;
}

void RobotController::updateStatistics(const uint32_t &bytesSent, const uint16_t &type) {
    #ifdef RRC_STATISTICS
        statistics.global.addBytesSent(bytesSent);
        statistics.stat_per_type[type].addBytesSent(bytesSent);
    #endif
}

std::string RobotController::sendRequest(const std::string& serializedMessage, const robot_remote_control::Transport::Flags &flags) {
    std::lock_guard<std::mutex> lock(commandTransportMutex);
    try {
        commandTransport->send(serializedMessage, flags);
    }catch (const std::exception &error) {
        connected.store(false);
        lostConnectionCallback(maxLatency);
        return "";
    }
    std::string replystr;

    requestTimer.start(maxLatency);
    while (commandTransport->receive(&replystr, flags) == 0 && !requestTimer.isExpired()) {
        // wait time depends on how long the transports recv blocks
        usleep(1000);
    }
    if (replystr.size() == 0 && requestTimer.isExpired()) {
        connected.store(false);
        lostConnectionCallback(lastConnectedTimer.getElapsedTime());
        return replystr;
    }
    lastConnectedTimer.start();
    connected.store(true);
    return replystr;
}

TelemetryMessageType RobotController::evaluateTelemetry(const std::string& reply) {
    uint16_t* type = reinterpret_cast<uint16_t*>(const_cast<char*>(reply.data()));

    std::string serializedMessage(reply.data()+sizeof(uint16_t), reply.size()-sizeof(uint16_t));

    TelemetryMessageType msgtype = (TelemetryMessageType)*type;

    updateStatistics(serializedMessage.size(), *type);

    // try to resolve through registered types
    std::shared_ptr<TelemetryAdderBase> adder = telemetryAdders[msgtype];
    if (adder.get()) {
        adder->addToTelemetryBuffer(msgtype, serializedMessage);
        return msgtype;
    }

    // handle special types

    switch (msgtype) {
        // multi values in single stream
        case SIMPLE_SENSOR_VALUE:       addToSimpleSensorBuffer(serializedMessage);
                                        return msgtype;

        case TELEMETRY_MESSAGE_TYPES_NUMBER:
        case NO_TELEMETRY_DATA:
        {
            return msgtype;
        }
        default:
        {
            throw std::range_error("message type " + std::to_string(msgtype) + " not registered, dropping telemetry");
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

    RingBufferAccess::pushData(simplesensorbuffer->lockedAccess().get()[data.id()], data, true);
}
