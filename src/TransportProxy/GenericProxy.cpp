#include "GenericProxy.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unistd.h>

namespace robot_remote_control {

GenericProxy::GenericProxy(const Type &type, TransportSharedPtr robotTransport, TransportSharedPtr controllerTransport) :
    UpdateThread(),
    type(type),
    robotTransport(robotTransport),
    controllerTransport(controllerTransport),
    timeout(2),
    longRequestTimeout(120),
    forwarded(0),
    errors(0) {

    if (!robotTransport || !controllerTransport) {
        throw std::runtime_error("GenericProxy: transports must not be null");
    }
    if (type == COMMANDS && !(robotTransport->supportsRobotControllerCommands() && controllerTransport->supportsControlledRobotCommands())) {
        throw std::runtime_error("GenericProxy: forwarding commands needs a robot transport that sends commands (e.g. zmq REQ) "
                                 "and a controller transport that receives them (e.g. zmq REP, http server)");
    }
    if (type == TELEMETRY && !(robotTransport->supportsRobotControllerTelemetry() && controllerTransport->supportsControlledRobotTelemetry())) {
        throw std::runtime_error("GenericProxy: forwarding telemetry needs a robot transport that receives telemetry (e.g. zmq SUB) "
                                 "and a controller transport that sends it (e.g. zmq PUB, websocket server)");
    }

    // payload types of the commands (version requests have none)
    registerCommandType<HeartBeat>(HEARTBEAT);
    registerCommandType<TelemetryRequest>(TELEMETRY_REQUEST);
    registerCommandType<FileRequest>(FILE_REQUEST);
    registerCommandType<LogLevelRequest>(LOG_LEVEL_SELECT);
    registerCommandType<Permission>(PERMISSION);
    registerCommandType<Pose>(TARGET_POSE_COMMAND);
    registerCommandType<Twist>(TWIST_COMMAND);
    registerCommandType<JointCommand>(JOINTS_COMMAND);
    registerCommandType<SimpleAction>(SIMPLE_ACTIONS_COMMAND);
    registerCommandType<ComplexAction>(COMPLEX_ACTION_COMMAND);
    registerCommandType<GoTo>(GOTO_COMMAND);
    registerCommandType<Poses>(ROBOT_TRAJECTORY_COMMAND);

    // telemetry types, like registered by the ControlledRobot
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
    registerTelemetryType<PermissionRequest>(PERMISSION_REQUEST);
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

GenericProxy::~GenericProxy() {
    stopUpdateThread();
}

void GenericProxy::update() {
    std::string message;
    if (type == COMMANDS) {
        while (controllerTransport->receiveMessage(&message, Transport::NOBLOCK)) {
            forwardCommand(message);
        }
    } else {
        while (robotTransport->receiveMessage(&message, Transport::NOBLOCK)) {
            forwardTelemetry(message);
        }
    }
}

void GenericProxy::forwardCommand(const std::string &request) {
    Serialization &controllerSerialization = controllerTransport->getSerialization();
    Serialization &robotSerialization = robotTransport->getSerialization();
    const bool convertMessages = controllerSerialization.getMode() != robotSerialization.getMode();

    ControlMessage message;
    if (!controllerSerialization.deserialize(request, &message)) {
        reportError("unable to parse a command");
        // the controller transport expects a reply for each request
        controllerTransport->send("");
        return;
    }
    const std::string &payload = controllerSerialization.getMode() == Serialization::JSON ? message.json() : message.data();

    // the type of a telemetry request determines the type of the reply
    uint16_t requestedTelemetry = NO_TELEMETRY_DATA;
    if (message.type() == TELEMETRY_REQUEST) {
        TelemetryRequest telemetryRequest;
        controllerSerialization.deserialize(payload, &telemetryRequest);
        requestedTelemetry = telemetryRequest.type();
    }

    std::string convertedRequest;
    if (convertMessages) {
        ControlMessage converted;
        converted.set_type(message.type());
        if (payload.size()) {
            auto prototype = commandTypes.find(message.type());
            std::string convertedPayload;
            if (prototype == commandTypes.end() ||
                !convert(*prototype->second, payload, &controllerSerialization, &robotSerialization, &convertedPayload)) {
                reportError("unable to convert command type " + std::to_string(message.type()));
                controllerTransport->send("");
                return;
            }
            robotSerialization.setSerialized(convertedPayload, &converted);
        }
        // the json serialization appends to the string, it has to be empty
        robotSerialization.serialize(converted, &convertedRequest);
    }
    const std::string &forwardedRequest = convertMessages ? convertedRequest : request;

    std::string reply;
    bool replied = false;
    try {
        robotTransport->send(forwardedRequest);
        // file transfers and maps may be big
        const bool longRequest = message.type() == FILE_REQUEST || requestedTelemetry == MAP;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(static_cast<int>((longRequest ? longRequestTimeout : timeout) * 1000));
        while (!(replied = robotTransport->receiveMessage(&reply, Transport::NOBLOCK))) {
            if (std::chrono::steady_clock::now() > deadline) {
                reportError("no reply from the robot");
                break;
            }
            usleep(1000);
        }
    } catch (const std::exception &error) {
        reportError(std::string("sending to the robot failed: ") + error.what());
    }
    if (!replied) {
        reply.clear();
    } else if (convertMessages && reply.size()) {
        // empty replies stay empty: no data (e.g. telemetry that was never set)
        reply = convertReply(message.type(), requestedTelemetry, reply);
    }
    // an empty reply tells the controller that there is no data
    controllerTransport->send(reply);
    if (replied) {
        ++forwarded;
    }
}

std::string GenericProxy::convertReply(const ControlMessageType &requestType, const uint16_t &requestedTelemetry, const std::string &reply) {
    Serialization &controllerSerialization = controllerTransport->getSerialization();
    Serialization &robotSerialization = robotTransport->getSerialization();

    std::shared_ptr<google::protobuf::Message> prototype;
    switch (requestType) {
        case PROTOCOL_VERSION:
        case LIBRARY_VERSION:
        case GIT_VERSION:
            // plain text
            return reply;
        case TELEMETRY_REQUEST: {
            auto telemetryType = telemetryTypes.find(requestedTelemetry);
            if (telemetryType == telemetryTypes.end()) {
                reportError("unable to convert telemetry type " + std::to_string(requestedTelemetry));
                return "";
            }
            prototype = telemetryType->second;
            break;
        }
        case FILE_REQUEST:
            prototype = std::make_shared<FolderTransfer>();
            break;
        default:
            prototype = std::make_shared<ControlMessageReply>();
    }
    std::string converted;
    if (!convert(*prototype, reply, &robotSerialization, &controllerSerialization, &converted)) {
        reportError("unable to convert the reply to command type " + std::to_string(requestType));
        return "";
    }
    return converted;
}

void GenericProxy::forwardTelemetry(const std::string &message) {
    Serialization &controllerSerialization = controllerTransport->getSerialization();
    Serialization &robotSerialization = robotTransport->getSerialization();

    std::string convertedMessage;
    const bool convertMessages = controllerSerialization.getMode() != robotSerialization.getMode();
    if (convertMessages) {
        TelemetryMessage telemetry;
        std::string payload;
        if (!robotSerialization.deserialize(message, &telemetry, &payload)) {
            reportError("unable to parse telemetry");
            return;
        }
        auto prototype = telemetryTypes.find(telemetry.type());
        std::string convertedPayload;
        if (prototype == telemetryTypes.end() ||
            !convert(*prototype->second, payload, &robotSerialization, &controllerSerialization, &convertedPayload)) {
            reportError("unable to convert telemetry type " + std::to_string(telemetry.type()));
            return;
        }
        TelemetryMessage converted;
        converted.set_type(telemetry.type());
        converted.set_channel(telemetry.channel());
        controllerSerialization.setSerialized(convertedPayload, &converted);
        // the json serialization appends to the string, it has to be empty
        controllerSerialization.serialize(converted, &convertedMessage);
    }

    try {
        controllerTransport->send(convertMessages ? convertedMessage : message);
        ++forwarded;
    } catch (const std::exception &error) {
        reportError(std::string("sending telemetry failed: ") + error.what());
    }
}

bool GenericProxy::convert(const google::protobuf::Message &prototype, const std::string &input, Serialization *from,
                           Serialization *to, std::string *output) {
    std::unique_ptr<google::protobuf::Message> message(prototype.New());
    if (!from->deserializeLongData(input, message.get())) {
        return false;
    }
    return to->serialize(*message, output);
}

void GenericProxy::reportError(const std::string &message) {
    ++errors;
    // print the first occurrence of an error, then less often
    const uint64_t count = ++reportedErrors[message];
    const double magnitude = std::log10(static_cast<double>(count));
    if (magnitude == std::floor(magnitude)) {
        printf("GenericProxy (%s): %s (%llu times)\n", type == COMMANDS ? "commands" : "telemetry", message.c_str(),
               static_cast<unsigned long long>(count));
    }
}

}  // namespace robot_remote_control
