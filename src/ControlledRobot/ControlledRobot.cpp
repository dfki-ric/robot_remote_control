#include "ControlledRobot.hpp"
#include <iostream>
#include <experimental/filesystem>
#include <fstream>
#include <sstream>

#ifdef ZLIB_FOUND
#include "../Tools/Compression.hpp"
#endif

namespace robot_remote_control {


ControlledRobot::ControlledRobot(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport, const size_t &buffersize):UpdateThread(),
    commandTransport(commandTransport),
    telemetryTransport(telemetryTransport),
    heartbeatAllowedLatency(0.1),
    connected(false),
    buffers(std::make_shared<TelemetryBuffer>()),
    logLevel(CUSTOM-1),
    receiveflags(Transport::NOBLOCK) {

    // init buffers for non-cast access in getters
    poseCommand = std::make_unique<CommandBuffer<Pose>>(buffersize);
    twistCommand = std::make_unique<CommandBuffer<Twist>>(buffersize);
    goToCommand = std::make_unique<CommandBuffer<GoTo>>(buffersize);
    simpleActionsCommand = std::make_unique<CommandBuffer<SimpleAction>>(buffersize);
    complexActionCommandBuffer = std::make_unique<CommandBuffer<ComplexAction>>(buffersize);
    jointsCommand = std::make_unique<CommandBuffer<JointCommand>>(buffersize);
    heartbeatCommand = std::make_unique<CommandBuffer<HeartBeat>>(1);
    permissionCommand = std::make_unique<CommandBuffer<Permission>>(1);
    robotTrajectoryCommand = std::make_unique<CommandBuffer<Poses>>(buffersize);

    // register command buffers
    registerCommandType(TARGET_POSE_COMMAND, poseCommand.get());
    registerCommandType(TWIST_COMMAND, twistCommand.get());
    registerCommandType(GOTO_COMMAND, goToCommand.get());
    registerCommandType(SIMPLE_ACTIONS_COMMAND, simpleActionsCommand.get());
    registerCommandType(COMPLEX_ACTION_COMMAND, complexActionCommandBuffer.get());
    registerCommandType(JOINTS_COMMAND, jointsCommand.get());
    registerCommandType(HEARTBEAT, heartbeatCommand.get());
    registerCommandType(PERMISSION, permissionCommand.get());
    registerCommandType(ROBOT_TRAJECTORY_COMMAND, robotTrajectoryCommand.get());


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
    registerTelemetryType<Map>(MAP);
    registerTelemetryType<Poses>(POSES);
    registerTelemetryType<Transforms>(TRANSFORMS);
    registerTelemetryType<PermissionRequest>(PERMISSION_REQUEST);  // no need to buffer, fills future
    registerTelemetryType<PointCloud>(POINTCLOUD);
    registerTelemetryType<IMU>(IMU_VALUES);
    registerTelemetryType<ContactPoints>(CONTACT_POINTS);
    registerTelemetryType<Twist>(CURRENT_TWIST);
    registerTelemetryType<Acceleration>(CURRENT_ACCELERATION);
    registerTelemetryType<CameraInformation>(CAMERA_INFORMATION);
    registerTelemetryType<Image>(IMAGE);
    registerTelemetryType<ImageLayers>(IMAGE_LAYERS);
    registerTelemetryType<Odometry>(ODOMETRY);
    registerTelemetryType<ControllableFrames>(CONTROLLABLE_FRAMES);
    registerTelemetryType<FileDefinition>(FILE_DEFINITION);
    registerTelemetryType<RobotModelInformation>(ROBOT_MODEL_INFORMATION);
    registerTelemetryType<InterfaceOptions>(INTERFACE_OPTIONS);
    registerTelemetryType<ChannelsDefinition>(CHANNELS_DEFINITION);
}

ControlledRobot::~ControlledRobot() {
    stopUpdateThread();
}

void ControlledRobot::update() {
    bool received = false;
    while (receiveRequest() != NO_CONTROL_DATA) {received = true;}
    if (received == true) {
        connected.store(true);
    }

    // if there are multiple connections with different frequencies it can happen that if the high frequency conenction is lost
    // and the last heartbeat message came from the low fewquency connection, the heartbeatExpiredCallback is called after
    // the low frequency timer is expired
    if (heartbeatCommand->hasNew() && heartbeatCommand->read(&heartbeatValues)) {
        connected.store(true);
        // printf("received new HB params %.2f, %.2f\n", heartbeatValues.heartbeatduration(), heartbeatValues.heartbeatlatency());
        heartbeatTimer.start(heartbeatValues.heartbeatduration() + heartbeatAllowedLatency);
    }
    if (heartbeatTimer.isExpired()) {
        connected.store(false);
        float elapsedTime = heartbeatTimer.getElapsedTime();
        if (heartbeatExpiredCallback != nullptr) {
            heartbeatExpiredCallback(elapsedTime);
        }
    }
}

void ControlledRobot::updateStatistics(const uint32_t &bytesSent, const uint16_t &type) {
    #ifdef RRC_STATISTICS
        statistics.global.addBytesSent(bytesSent);
        statistics.stat_per_type[type].addBytesSent(bytesSent);
    #endif
}

ControlMessageType ControlledRobot::receiveRequest() {
    std::string msg;
    int result = commandTransport->receive(&msg, receiveflags);
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
            return handleTelemetryRequest(serializedMessage, commandTransport);
        }
        case MAP_REQUEST: {
            return handleMapRequest(serializedMessage, commandTransport);
        }
        case LOG_LEVEL_SELECT: {
            logLevel = *reinterpret_cast<uint16_t*>(const_cast<char*>(serializedMessage.data()));
            commandTransport->send(serializeControlMessageType(LOG_LEVEL_SELECT));
            return LOG_LEVEL_SELECT;
        }
        case PERMISSION: {
            return handlePermissionRequest(serializedMessage, commandTransport);
        }
        case FILE_REQUEST: {
            return handleFileRequest(serializedMessage, commandTransport);
        }
        default: {
            return handleCommandRequest(msgtype, serializedMessage, commandTransport);
        }
    }
}

void ControlledRobot::notifyCommandCallbacks(const uint16_t &type) {
    auto callCb = [&](const std::function<void(const uint16_t &type)> &cb){cb(type);};
    std::for_each(commandCallbacks.begin(), commandCallbacks.end(), callCb);
}


int ControlledRobot::setRobotState(const std::string& state) {
    RobotState protostate;
    *protostate.add_state() = state;
    return sendTelemetry(protostate, ROBOT_STATE, false, 0);
}

int ControlledRobot::setRobotState(const std::vector<std::string> state) {
    RobotState protostate;
    for (const std::string &line : state) {
        *protostate.add_state() = line;
    }
    return sendTelemetry(protostate, ROBOT_STATE, false, 0);
}

int ControlledRobot::setRobotState(const RobotState& state) {
    return sendTelemetry(state, ROBOT_STATE, false, 0);
}

int ControlledRobot::setLogMessage(enum LogLevel lvl, const std::string& message) {
    if (lvl <= logLevel || lvl >= CUSTOM) {
        LogMessage msg;
        msg.set_level(lvl);
        msg.set_message(message);
        return sendTelemetry(msg, LOG_MESSAGE, false, 0);
    }
    return -1;
}

int ControlledRobot::setLogMessage(const LogMessage& log_message) {
    if (log_message.level() <= logLevel || log_message.level() >= CUSTOM) {
        return sendTelemetry(log_message, LOG_MESSAGE, false, 0);
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


bool ControlledRobot::loadFile(File* file, const std::string &path, bool compressed) {
    file->set_path(path);
    // read file (if it is and directory, no data is set)
    std::ifstream in(path, std::ios::in | std::ios::binary);
    if (in) {
        std::stringstream filestr;
        filestr << in.rdbuf();
        in.close();
        #ifdef ZLIB_FOUND
            if (compressed) {
                std::string compressed;
                Compression::compressString(filestr.str(), &compressed);
                file->set_data(compressed);
            } else {
                file->set_data(filestr.str());
            }
        #else
            file->set_data(filestr.str());
        #endif
        return true;
    }
    return false;
}

bool ControlledRobot::loadFolder(Folder* folder, const std::string &path, bool compressed) {
    try {
        for (const auto & entry : std::experimental::filesystem::recursive_directory_iterator(path)) {
            File* file = folder->add_file();
            loadFile(file, entry.path(), compressed);
        }
        folder->set_compressed(compressed);
    } catch (const std::experimental::filesystem::v1::__cxx11::filesystem_error &e) {
        printf("%s\n", e.what());
        // set "someting non-default" to actially send values
        folder->set_identifier(e.what());
        return false;
    }
    return true;
}


ControlMessageType ControlledRobot::handleTelemetryRequest(const std::string& serializedMessage, robot_remote_control::TransportSharedPtr commandTransport) {
    uint16_t* requestedtype = reinterpret_cast<uint16_t*>(const_cast<char*>(serializedMessage.data()));
    uint8_t* requestedchannel = reinterpret_cast<uint8_t*>(const_cast<char*>(serializedMessage.data()+sizeof(uint16_t)));
    //TODO channel
    TelemetryMessageType type = (TelemetryMessageType) *requestedtype;
    std::string reply = buffers->peekSerialized(type, *requestedchannel);
    commandTransport->send(reply);
    return TELEMETRY_REQUEST;
}

ControlMessageType ControlledRobot::handleMapRequest(const std::string& serializedMessage, robot_remote_control::TransportSharedPtr commandTransport) {
    uint16_t* requestedMap = reinterpret_cast<uint16_t*>(const_cast<char*>(serializedMessage.data()));
    std::string map;
    // get map
    {
        auto lockedAccess = mapBuffer.lockedAccess();
        if (*requestedMap < lockedAccess.get().size()) {
            RingBufferAccess::peekData(lockedAccess.get()[*requestedMap], &map);
        }
    }
    commandTransport->send(map);
    return MAP_REQUEST;
}

ControlMessageType ControlledRobot::handlePermissionRequest(const std::string& serializedMessage, robot_remote_control::TransportSharedPtr commandTransport) {
    Permission perm;
    perm.ParseFromString(serializedMessage);
    std::promise<bool> &promise = pendingPermissionRequests[perm.requestuid()];
    try {
        promise.set_value(perm.granted());
    } catch (const std::future_error &e) {
        printf("%s\n", e.what());
    }
    commandTransport->send(serializeControlMessageType(PERMISSION));
    return PERMISSION;
}

ControlMessageType ControlledRobot::handleFileRequest(const std::string& serializedMessage, robot_remote_control::TransportSharedPtr commandTransport) {
    FileRequest request;
    request.ParseFromString(serializedMessage);
    Folder folder;
    std::string buf;
    int index = -1;
    for (int i = 0; i < files.file().size(); ++i) {
        if (files.file(i).identifier() == request.identifier()) {
            index = i;
            break;
        }
    }

    #ifndef ZLIB_FOUND
        printf("zlib for compression not available, sending uncompressed files\n");
        request.set_compressed(false);
    #endif
    if (index >= 0 && index < files.file().size()) {
        bool isFolder = files.isfolder(index);
        File filedef = files.file(index);
        // todo: read folderfolder
        if (isFolder) {
            loadFolder(&folder, filedef.path(), request.compressed());
        } else {
            File* file = folder.add_file();
            loadFile(file, filedef.path(), request.compressed());
            folder.set_compressed(request.compressed());
        }
    } else {
        printf("requested file '%s' undefined, sending empty folder\n", request.identifier().c_str());
        folder.set_identifier("file/folder :" + request.identifier() + " undefined");
    }
    folder.SerializeToString(&buf);
    commandTransport->send(buf);
    return FILE_REQUEST;
}

ControlMessageType ControlledRobot::handleCommandRequest(const ControlMessageType &msgtype, const std::string& serializedMessage, robot_remote_control::TransportSharedPtr commandTransport) {
    CommandBufferBase * cmdbuffer = commandbuffers[msgtype];
    if (cmdbuffer) {
        if (!cmdbuffer->write(serializedMessage)) {
            printf("unable to parse message of type %i in %s:%i\n", msgtype, __FILE__, __LINE__);
            commandTransport->send(serializeControlMessageType(NO_CONTROL_DATA));
            return NO_CONTROL_DATA;
        }
        commandTransport->send(serializeControlMessageType(msgtype));
        notifyCommandCallbacks(msgtype);
        return msgtype;
    } else {
        commandTransport->send(serializeControlMessageType(NO_CONTROL_DATA));
        return msgtype;
    }
}



}  // namespace robot_remote_control
