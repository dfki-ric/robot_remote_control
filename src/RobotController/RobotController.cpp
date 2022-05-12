#include "RobotController.hpp"
#include <iostream>
#include <memory>
#include <unistd.h>
#include <stdexcept>
#include <google/protobuf/io/coded_stream.h>

#ifdef ZLIB_FOUND
#include "../Tools/Compression.hpp"
#endif


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
        registerTelemetryType<SimpleSensor>(SIMPLE_SENSOR_VALUE, buffersize);  // this is a configuration, so no bigger buffer needed
        registerTelemetryType<WrenchState>(WRENCH_STATE, buffersize);
        registerTelemetryType<MapsDefinition>(MAPS_DEFINITION, 1);
        registerTelemetryType<Map>(MAP, 1);
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
        registerTelemetryType<FileDefinition>(FILE_DEFINITION, 1);

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

bool RobotController::setSingleTelemetryBufferOverwrite(TelemetryMessageType type, bool overwrite) {
    std::shared_ptr<TelemetryAdderBase> adder =  telemetryAdders[type];
    if (adder.get()) {
        adder->setOverwrite(overwrite);
        return true;
    }
    return false;
}

bool RobotController::setSingleTelemetryBufferSize(TelemetryMessageType type, uint16_t newsize) {
    auto lockedTelemetryBuffers = buffers->lockedAccess();
    std::shared_ptr <RingBufferBase> buffer = lockedTelemetryBuffers.get()[type];
    if (buffer.get()) {
        buffer->resize(newsize);
        return true;
    }
    return false;
}

uint32_t RobotController::getTelemetryBufferDataSize(const TelemetryMessageType &type) {
    return buffers->lockedAccess().get()[type]->size();
}

size_t RobotController::getDroppedTelemetry(const TelemetryMessageType &type) {
    return buffers->lockedAccess().get()[type]->dropped();
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
    return true;
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
            }
            heartBeatTimer.start(heartBeatDuration);
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
    } catch (const std::exception &error) {
        connected.store(false);
        lostConnectionCallback(maxLatency);
        return "";
    }
    std::string replystr;

    requestTimer.start(maxLatency);

    try {
        while (commandTransport->receive(&replystr, flags) == 0 && !requestTimer.isExpired()) {
            // wait time depends on how long the transports recv blocks
            usleep(1000);
        }
    } catch (const std::exception &error) {
        connected.store(false);
        lostConnectionCallback(maxLatency);
        return "";
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
            // try to resolve through registered types
            std::shared_ptr<TelemetryAdderBase> adder = telemetryAdders[msgtype];
            if (adder.get()) {
                adder->addToTelemetryBuffer(msgtype, serializedMessage);
                return msgtype;
            } else {
                throw std::range_error("message type " + std::to_string(msgtype) + " not registered, dropping telemetry");
            }
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
    // also push the data to the "traditional" buffer
    RingBufferAccess::pushData(buffers->lockedAccess().get()[SIMPLE_SENSOR_VALUE], data, true);

}

bool RobotController::requestFile(const std::string &identifier, const bool &compressed,  const std::string targetpath) {
    std::string buffer;
    FileRequest request;
    request.set_identifier(identifier);

    #ifdef ZLIB_FOUND
        request.set_compressed(compressed);
    #else
        printf("zlib for compression not available, requesting files uncompressed\n");
        request.set_compressed(false);
    #endif

    Folder folder;

    bool result = requestProtobuf(request, &folder, FILE_REQUEST);
    if (result) {
        if (folder.file().size() == 0) {
            // no files defined in ControlledRobot
            folder.PrintDebugString();
            return false;
        }
        std::experimental::filesystem::create_directories(targetpath);
        for (auto &file : folder.file()) {
            std::string filename = targetpath  + "/" + file.path();

            if (file.data().size() == 0) {
                // is directory
                std::experimental::filesystem::create_directories(filename);
            } else {
                // is file
                std::experimental::filesystem::path path(filename);
                std::experimental::filesystem::create_directories(path.parent_path());
                // printf("file %s\n dir %s\n", filename.c_str(), path.parent_path().c_str());
                std::ofstream out(filename, std::ios::out | std::ios::binary);
                if (out) {
                    #ifdef ZLIB_FOUND
                        if (folder.compressed()) {
                            std::string decompressed;
                            Compression::decompressString(file.data(), &decompressed);
                            out.write(decompressed.data(), decompressed.size());
                        } else {
                            out.write(file.data().data(), file.data().size());
                        }
                    #else
                        out.write(file.data().data(), file.data().size());
                    #endif
                    out.close();
                }
            }
        }
    }
    return result;
}
