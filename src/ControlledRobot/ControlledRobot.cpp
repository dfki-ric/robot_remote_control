#include "ControlledRobot.hpp"
#include <iostream>
#include <experimental/filesystem>
#include <fstream>
#include <sstream>

#ifdef ZLIB_FOUND
#include "../Tools/Compression.hpp"
#endif

#include "ProtocolVersion.hpp"
#include "LibraryVersion.hpp"

namespace robot_remote_control {


ControlledRobot::ControlledRobot(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport, const size_t &buffersize):UpdateThread(),
    commandTransport(commandTransport),
    telemetryTransport(telemetryTransport),
    heartbeatAllowedLatency(0.1),
    connectionLostCallbackInterval(1),
    connected(false),
    buffers(std::make_shared<TelemetryBuffer>()),
    logLevel(CUSTOM-1),
    receiveflags(Transport::NOBLOCK) {

    // init buffers for non-cast access in getters
    protocolVersionBuf = std::make_unique<MessageIdCommandBuffer>(1);
    libraryVersionBuf = std::make_unique<MessageIdCommandBuffer>(1);
    gitVersionBuf = std::make_unique<MessageIdCommandBuffer>(1);
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
    registerCommandType(PROTOCOL_VERSION, protocolVersionBuf.get());
    registerCommandType(LIBRARY_VERSION, libraryVersionBuf.get());
    registerCommandType(GIT_VERSION, gitVersionBuf.get());

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
    registerTelemetryType<SimpleSensor>(SIMPLE_SENSOR);
    registerTelemetryType<WrenchState>(WRENCH_STATE);
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
    registerTelemetryType<LaserScan>(LASER_SCAN);
}

ControlledRobot::~ControlledRobot() {
    stopUpdateThread();
}

void ControlledRobot::update() {
    bool received = false;
    while (receiveRequest() != NO_CONTROL_DATA) {received = true;}
    if (received == true) {
        if (!connected) {
            if (connectedCallback != nullptr) {
                connectedCallback();
            }
        }
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
        if (connected) {
            connectionLostCallbackTimer.start(0);
        }
        connected.store(false);
        if (heartbeatExpiredCallback != nullptr) {
            if (connectionLostCallbackTimer.isExpired()){
                float elapsedTime = heartbeatTimer.getElapsedTime();
                heartbeatExpiredCallback(elapsedTime);
                connectionLostCallbackTimer.start(connectionLostCallbackInterval);
            }
        }
    }
}

void ControlledRobot::updateStatistics(const uint32_t &bytesSent, const MessageId &type) {
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

    ControlMessage controlMessage;
    std::string serializedMessage;

    serialization.deserialize(request, &controlMessage);

    if (controlMessage.json() != "") {
        serializedMessage = controlMessage.json();
    }else{
        serializedMessage = controlMessage.data();
    }
    
    ControlMessageType msgtype = controlMessage.type();


    switch (msgtype) {
        case PROTOCOL_VERSION:
        case LIBRARY_VERSION:
        case GIT_VERSION: {
            return handleVersionRequest(msgtype, commandTransport);
        }
        case TELEMETRY_REQUEST: {
            return handleTelemetryRequest(serializedMessage, commandTransport);
        }
        case LOG_LEVEL_SELECT: {
            LogLevelRequest req;
            serialization.deserialize(serializedMessage, &req);

            logLevel = req.level();
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

void ControlledRobot::notifyCommandCallbacks(const MessageId &type) {
    auto callCb = [&](const std::function<void(const MessageId &type)> &cb){cb(type);};
    std::for_each(commandCallbacks.begin(), commandCallbacks.end(), callCb);
}

int ControlledRobot::initFiles(const FileDefinition& files) {
    // store private version
    this->internal_files.MergeFrom(files);
    // remove local paths from remote version
    FileDefinition remote;
    remote.CopyFrom(files);
    for (auto& file : *remote.mutable_file()) {
        if (file.remote_path().size()) {
            // hide local path from RobotController
            file.set_path(file.remote_path());
            file.set_remote_path("");
        }
    }
    // update remote version
    this->remote_files.MergeFrom(remote);
    // initialize buffer for requests
    return sendTelemetry(this->remote_files, FILE_DEFINITION, true, 0);
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

std::string ControlledRobot::serializeControlMessageType(const ControlMessageType& type) {
    ControlMessageReply reply;
    reply.set_type(type);
    std::string buf;
    reply.SerializeToString(&buf);
    return buf;
}


bool ControlledRobot::loadFile(FileTransfer* file, const std::string &path, bool compressed, const std::string &remotePath) {
    // if remote_path is set in def, override file path
    if (remotePath == "") {
        file->set_path(path);
    } else {
        file->set_path(remotePath);
    }

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

bool ControlledRobot::loadFolder(FolderTransfer* folder, const std::string &path, bool compressed, const std::string &remotePath) {
    try {
        for (const auto & entry : std::experimental::filesystem::recursive_directory_iterator(path)) {
            FileTransfer* file = folder->add_file();
            std::string remoteFilename = entry.path();
            if (remotePath != "") {
                // the folder part on the remote should be different
                // get the local part
                std::string localPath = remoteFilename.substr(path.size(),remoteFilename.size()-path.size());
                printf("l: %s\n", localPath.c_str());
                //construct new remote path
                remoteFilename = remotePath + localPath;
            }
            loadFile(file, entry.path(), compressed, remoteFilename);
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
    TelemetryRequest request;

    serialization.deserialize(serializedMessage, &request);
    std::string reply = buffers->peekSerialized(request.type(), request.channel(), serialization.getMode());
    commandTransport->send(reply);
    return TELEMETRY_REQUEST;
}

ControlMessageType ControlledRobot::handlePermissionRequest(const std::string& serializedMessage, robot_remote_control::TransportSharedPtr commandTransport) {
    Permission perm;

    serialization.deserialize(serializedMessage, &perm);

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

    serialization.deserialize(serializedMessage, &request);

    FolderTransfer folder;
    std::string buf;
    int index = -1;
    for (int i = 0; i < internal_files.file().size(); ++i) {
        if (internal_files.file(i).identifier() == request.identifier()) {
            index = i;
            break;
        }
    }

    #ifndef ZLIB_FOUND
        printf("zlib for compression not available, sending uncompressed files\n");
        request.set_compressed(false);
    #endif
    if (index >= 0 && index < internal_files.file().size()) {
        bool isFolder = internal_files.isfolder(index);
        File filedef = internal_files.file(index);
        // todo: read folderfolder
        if (isFolder) {
            loadFolder(&folder, filedef.path(), request.compressed(), filedef.remote_path());
        } else {
            FileTransfer* file = folder.add_file();
            loadFile(file, filedef.path(), request.compressed(), filedef.remote_path());
            folder.set_compressed(request.compressed());
        }
    } else {
        printf("requested file '%s' undefined, sending empty folder\n", request.identifier().c_str());
        folder.set_identifier("file/folder :" + request.identifier() + " undefined");
    }
    serialization.serialize(folder, &buf);


    commandTransport->send(buf);
    return FILE_REQUEST;
}

ControlMessageType ControlledRobot::handleCommandRequest(const ControlMessageType &msgtype, const std::string& serializedMessage, robot_remote_control::TransportSharedPtr commandTransport) {
    CommandBufferBase * cmdbuffer = commandbuffers[msgtype];
    if (cmdbuffer) {
        if (!cmdbuffer->write(serializedMessage, serialization.getMode())) {
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

ControlMessageType ControlledRobot::handleVersionRequest(const MessageId& msgid, TransportSharedPtr commandTransport) {
    std::string msg = "";
    MessageIdCommandBuffer* cmdbuffer = dynamic_cast<MessageIdCommandBuffer*>(commandbuffers[msgid]);
    if (cmdbuffer) {
        cmdbuffer->write(msgid);
    }
    std::string version;

    switch (msgid) {
        case PROTOCOL_VERSION: version = protocolVersion(); break;
        case LIBRARY_VERSION:  version = libraryVersion(); break;
        case GIT_VERSION:      version = gitVersion(); break;
    }
    commandTransport->send(version);
    return ControlMessageType(msgid);
}

std::string ControlledRobot::protocolVersion() {
    return PROTOCOL_VERSION_CHECKSUM;
}
std::string ControlledRobot::libraryVersion() {
    return LIBRARY_VERSION_STRING;
}
std::string ControlledRobot::gitVersion() {
    return GIT_COMMIT_ID;
}


}  // namespace robot_remote_control
