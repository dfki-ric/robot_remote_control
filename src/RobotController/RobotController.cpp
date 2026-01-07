#include "RobotController.hpp"
#include <iostream>
#include <memory>
#include <unistd.h>
#include <stdexcept>

#include "ProtocolVersion.hpp"
#include "LibraryVersion.hpp"

#ifdef ZLIB_FOUND
#include "../Tools/Compression.hpp"
#endif


using namespace robot_remote_control;


RobotController::RobotController(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport, const size_t &buffersize, const float &maxLatency):UpdateThread(),
    commandTransport(commandTransport),
    telemetryTransport(telemetryTransport),
    heartBeatDuration(0),
    heartBeatRoundTripTime(0),
    maxLatency(maxLatency),
    buffers(std::make_shared<TelemetryBuffer>()),
    connected(false),
    useJSON(true) {
        registerTelemetryType<Pose>(CURRENT_POSE, buffersize);
        registerTelemetryType<JointState>(JOINT_STATE, buffersize);
        registerTelemetryType<JointState>(CONTROLLABLE_JOINTS, buffersize);
        registerTelemetryType<SimpleActions>(SIMPLE_ACTIONS, 1);  // this is a configuration, so no bigger buffer needed
        registerTelemetryType<ComplexActions>(COMPLEX_ACTIONS, 1);  // this is a configuration, so no bigger buffer needed
        registerTelemetryType<RobotName>(ROBOT_NAME, 1);  // this is a configuration, so no bigger buffer needed
        registerTelemetryType<RobotState>(ROBOT_STATE, buffersize);
        registerTelemetryType<LogMessage>(LOG_MESSAGE, buffersize);
        registerTelemetryType<VideoStreams>(VIDEO_STREAMS, 1);  // this is a configuration, so no bigger buffer needed
        registerTelemetryType<SimpleSensor>(SIMPLE_SENSOR, buffersize);  // this is a configuration, so no bigger buffer needed
        registerTelemetryType<WrenchState>(WRENCH_STATE, buffersize);
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
        registerTelemetryType<RobotModelInformation>(ROBOT_MODEL_INFORMATION, 1);
        registerTelemetryType<InterfaceOptions>(INTERFACE_OPTIONS, 1);
        registerTelemetryType<ChannelsDefinition>(CHANNELS_DEFINITION, 1);
        registerTelemetryType<LaserScan>(LASER_SCAN, buffersize);

        // #ifdef RRC_STATISTICS
        //     // add names to buffer, this types have aspecial treatment, the should not be registered
        //     MapsDefinition mapsDefinition;
        //     statistics.names[MAPS_DEFINITION] = mapsDefinition.GetTypeName();
        //     Map map;
        //     statistics.names[MAP] = map.GetTypeName();
        // #endif

        for (auto& chan : messageChannels) {
            chan = 0;
        }

        lostConnectionCallback = [&](const float& time){
            printf("lost connection to robot, no reply for %f seconds\n", time);
        };

        connectedCallback = [&](){
            printf("connected to robot\n");
        };
}

RobotController::~RobotController() {
    stopUpdateThread();
}

bool RobotController::checkProtocolVersion() {
    std::string remote_ver = requestProtocolVersion();
    if (remote_ver != PROTOCOL_VERSION_CHECKSUM) {
        printf("protocol version mismatch, consider using the same version on both sides if your calls do not work\n");
        return false;
    }
    return true;
}

std::string RobotController::requestProtocolVersion() {
    std::string buf;
    buf.resize(sizeof(MessageId));
    MessageId* data = reinterpret_cast<MessageId*>(const_cast<char*>(buf.data()));
    *data = PROTOCOL_VERSION;
    return sendRequest(buf);
}

std::string RobotController::protocolVersion() {
    return PROTOCOL_VERSION_CHECKSUM;
}

bool RobotController::checkLibraryVersion() {
    std::string remote_ver = requestLibraryVersion();
    if (remote_ver != LIBRARY_VERSION_STRING) {
        printf("library version mismatch (%s : %s), consider using the same version on both sides if your calls do not work\n", remote_ver.c_str(), LIBRARY_VERSION_STRING);
        return false;
    }
    return true;
}

std::string RobotController::requestLibraryVersion() {
    std::string buf;
    buf.resize(sizeof(MessageId));
    MessageId* data = reinterpret_cast<MessageId*>(const_cast<char*>(buf.data()));
    *data = LIBRARY_VERSION;
    return sendRequest(buf);
}

std::string RobotController::libraryVersion() {
    return LIBRARY_VERSION_STRING;
}

bool RobotController::checkGitVersion() {
    std::string remote_ver = requestGitVersion();
    if (remote_ver != GIT_COMMIT_ID) {
        printf("git version mismatch, consider using the same version on both sides if your calls do not work\n");
        return false;
    }
    return true;
}

std::string RobotController::requestGitVersion() {
    std::string buf;
    buf.resize(sizeof(MessageId));
    MessageId* data = reinterpret_cast<MessageId*>(const_cast<char*>(buf.data()));
    *data = GIT_VERSION;
    return sendRequest(buf);
}

std::string RobotController::gitVersion() {
    return GIT_COMMIT_ID;
}



bool RobotController::setSingleTelemetryBufferOverwrite(TelemetryMessageType type, bool overwrite, const ChannelId &channel) {
    std::shared_ptr<TelemetryAdderBase> adder =  telemetryAdders[type];
    if (adder.get()) {
        adder->setOverwrite(overwrite);
        return true;
    }
    return false;
}

void RobotController::setTelemetryBufferOverwrite(bool overwrite, const ChannelId &channel) {
    for (auto &adder : telemetryAdders) {
        if (adder.get()) {
            adder->setOverwrite(overwrite);
        }
    }
}

bool RobotController::setSingleTelemetryBufferSize(TelemetryMessageType type, size_t newsize, const ChannelId &channel) {
    auto lockedTelemetryBuffers = buffers->lockedAccess();
    std::shared_ptr <RingBufferBase> buffer = lockedTelemetryBuffers.get()[type][channel];
    if (buffer.get()) {
        buffer->resize(newsize);
        return true;
    }
    return false;
}

uint32_t RobotController::getTelemetryBufferDataSize(const TelemetryMessageType &type, const ChannelId &channel) {
    return buffers->lockedAccess().get()[type][channel]->size();
}

size_t RobotController::getDroppedTelemetry(const TelemetryMessageType &type, const ChannelId &channel) {
    return buffers->lockedAccess().get()[type][channel]->dropped();
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

void RobotController::setLogLevel(const LogLevelId &level) {
    std::string buf;
    LogLevelRequest levelrequest;
    levelrequest.set_level(static_cast<LogLevel>(level));
    ControlMessage controlMessage = initControlMessage(LOG_LEVEL_SELECT, levelrequest);
    controlMessage.SerializeToString(&buf);
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
            MessageId type = evaluateTelemetry(buf);
            if (type != NO_TELEMETRY_DATA) {
                auto callCb = [&](const std::function<void(const MessageId & type)> &cb){cb(type);};
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
                heartBeatRoundTripTime.store(time);
            }
            heartBeatTimer.start(heartBeatDuration);
        }
    }
}

bool RobotController::requestMap(Map *map, const ChannelId &channel, const float &overrideMaxLatency){
    std::string replybuf;
    bool result = requestBinary(MAP, &replybuf, TELEMETRY_REQUEST, channel, overrideMaxLatency);

     if (useJSON) {
        google::protobuf::util::JsonStringToMessage(replybuf, map);
     } else {
        google::protobuf::io::CodedInputStream cistream(reinterpret_cast<const uint8_t *>(replybuf.data()), replybuf.size());
        cistream.SetTotalBytesLimit(replybuf.size());
        map->ParseFromCodedStream(&cistream);
    }


    return result;
}

void RobotController::updateStatistics(const uint32_t &bytesSent, const MessageId &type) {
    #ifdef RRC_STATISTICS
        statistics.global.addBytesSent(bytesSent);
        statistics.stat_per_type[type].addBytesSent(bytesSent);
    #endif
}

std::string RobotController::sendRequest(const std::string& serializedMessage, const float &overrideMaxLatency, const robot_remote_control::Transport::Flags &flags) {
    std::lock_guard<std::mutex> lock(commandTransportMutex);

    float currentMaxLatency = maxLatency;
    if (overrideMaxLatency > 0) {
        currentMaxLatency = overrideMaxLatency;
    }

    try {
        commandTransport->send(serializedMessage, flags);
    } catch (const std::exception &error) {
        connected.store(false);
        lostConnectionCallback(lastConnectedTimer.getElapsedTime());
        return "";
    }
    std::string replystr;

    requestTimer.start(currentMaxLatency);

    try {
        while (commandTransport->receive(&replystr, flags) == 0 && !requestTimer.isExpired()) {
            // wait time depends on how long the transports recv blocks
            usleep(1000);
        }
    } catch (const std::exception &error) {
        connected.store(false);
        lostConnectionCallback(lastConnectedTimer.getElapsedTime());
        return "";
    }
    if (replystr.size() == 0 && requestTimer.isExpired()) {
        connected.store(false);
        lostConnectionCallback(lastConnectedTimer.getElapsedTime());
        return replystr;
    }
    lastConnectedTimer.start();

    if (!connected) {
        if (connectedCallback != nullptr) {
            connectedCallback();
        }
    }

    connected.store(true);
    return replystr;
}

TelemetryMessageType RobotController::evaluateTelemetry(const std::string& reply) {

    TelemetryMessage telemetryMessage;

    if (useJSON) {
        google::protobuf::util::JsonStringToMessage(reply, &telemetryMessage);
    }else{
        telemetryMessage.ParseFromString(reply);
    }
    


    ChannelId channel = telemetryMessage.channel();
    TelemetryMessageType msgtype = telemetryMessage.type();
    
    std::string serializedMessage;
    if (useJSON) {
        serializedMessage = telemetryMessage.json();
    } else {
        serializedMessage = telemetryMessage.data();
    }


    updateStatistics(serializedMessage.size(), msgtype);

    // handle special types
    switch (msgtype) {
        case TELEMETRY_MESSAGE_TYPES_NUMBER:
        case NO_TELEMETRY_DATA:
        {
            return msgtype;
        }
        default:
        {
            if (channel != 0) {
                if (!buffers->hasChannelBuffer(msgtype,channel)) {
                    //TODO: let user pre-define buffers for channels and also defiene buffersize after creation
                    buffers->addChannelBuffer(msgtype,channel, 10);
                }
            }
            // try to resolve through registered types
            std::shared_ptr<TelemetryAdderBase> adder = telemetryAdders[msgtype];
            if (adder.get()) {
                adder->addToTelemetryBuffer(msgtype, serializedMessage, channel, useJSON);
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

bool RobotController::requestBinary(const TelemetryMessageType &type, std::string *result, const ControlMessageType &requestType, const ChannelId &channel, const float &overrideMaxLatency) {
    std::string request;

    TelemetryRequest telemetryRequest;
    telemetryRequest.set_type(type);
    telemetryRequest.set_channel(channel);
    telemetryRequest.SerializeToString(&request);

    return requestBinary(request, result, requestType, overrideMaxLatency);
}

bool RobotController::requestBinary(const std::string &request, std::string *result, const ControlMessageType &requestType, const float &overrideMaxLatency) {
    std::string buf;
    ControlMessage controlmessage = initControlMessage(requestType, request);
    
    controlmessage.SerializeToString(&buf);
    
    *result = sendRequest(buf, overrideMaxLatency);
    return (result->size() > 0) ? true : false;
}


bool RobotController::requestFile(const std::string &identifier, const bool &compressed,  const std::string targetpath, const float &overrideMaxLatency) {
    std::string buffer;
    FileRequest request;
    request.set_identifier(identifier);

    #ifdef ZLIB_FOUND
        request.set_compressed(compressed);
    #else
        printf("zlib for compression not available, requesting files uncompressed\n");
        request.set_compressed(false);
    #endif

    FolderTransfer folder;

    bool result = requestProtobuf(request, &folder, FILE_REQUEST, overrideMaxLatency);
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

std::pair<std::string, std::string> RobotController::requestRobotModel(const std::string &targetfolder, const float &overrideMaxLatency) {
    RobotModelInformation model;
    if (requestTelemetry(ROBOT_MODEL_INFORMATION, &model, 0)) {
        if (requestFile(model.filedef().file(0).identifier(), true, targetfolder, overrideMaxLatency)) {
            return {model.filedef().file(0).path(), model.modelfilename()};
        }
    }
    return {"",""};
}

ControlMessage RobotController::initControlMessage(const ControlMessageType &type, const std::string &data) {
    ControlMessage controlMessage;
    controlMessage.set_type(type);
    controlMessage.set_data(data);
    return controlMessage;
}
