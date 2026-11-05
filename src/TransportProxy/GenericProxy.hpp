#pragma once

#include <map>
#include <memory>
#include <string>
#include <atomic>

#include "../Transports/Transport.hpp"
#include "../UpdateThread/UpdateThread.hpp"

namespace robot_remote_control {

/**
 * @brief Forwards the messages of a robot_remote_control connection between two transports, e.g. to let
 * controllers connect with other transports than the robot offers (a web interface using http and
 * websockets with json to a robot using zmq).
 *
 * One proxy forwards one connection:
 * - COMMANDS: requests received from controllers by the controllerTransport (a robot side transport, e.g.
 *   TransportHttp, zmq REP) are sent to the robot with the robotTransport (a controller side transport,
 *   e.g. zmq REQ), the replies of the robot are sent back.
 * - TELEMETRY: telemetry received from the robot by the robotTransport (e.g. zmq SUB) is sent to the
 *   controllers with the controllerTransport (e.g. websocket server, zmq PUB).
 *
 * When the transports use different serialization modes (e.g. binary for zmq, json for http), the messages
 * are converted. This needs their types: all types of robot_remote_control are registered, libraries that
 * extend the protocol can add theirs with registerCommandType() and registerTelemetryType().
 * When both use the same mode, the messages are forwarded without parsing them.
 */
class GenericProxy : public UpdateThread {
 public:
    enum Type {COMMANDS, TELEMETRY};

    /**
     * @param type the connection to forward
     * @param robotTransport connected to the robot
     * @param controllerTransport offered to the controllers
     * @throws std::runtime_error if the transports do not support the type
     */
    GenericProxy(const Type &type, TransportSharedPtr robotTransport, TransportSharedPtr controllerTransport);

    virtual ~GenericProxy();

    void update() override;

    /**
     * @brief how long to wait for the reply of the robot, controllers get an empty reply after the timeout
     *
     * @param seconds for most commands
     * @param longRequestSeconds for file and map requests
     */
    void setTimeout(const float &seconds, const float &longRequestSeconds = 120) {
        timeout = seconds;
        longRequestTimeout = longRequestSeconds;
    }

    /**
     * @brief register the message type of a ControlMessageType, needed to convert the command
     */
    template <class PROTO> void registerCommandType(const uint16_t &type) {
        commandTypes[type] = std::make_shared<PROTO>();
    }

    /**
     * @brief register the message type of a TelemetryMessageType, needed to convert the telemetry
     * (and the replies of telemetry requests)
     */
    template <class PROTO> void registerTelemetryType(const uint16_t &type) {
        telemetryTypes[type] = std::make_shared<PROTO>();
    }

    /**
     * @brief number of forwarded messages (commands or telemetry)
     */
    uint64_t getForwardedCount() const {
        return forwarded.load();
    }

    /**
     * @brief number of messages that could not be forwarded (unknown types, timeouts, transport errors)
     */
    uint64_t getErrorCount() const {
        return errors.load();
    }

 private:
    void forwardCommand(const std::string &request);
    void forwardTelemetry(const std::string &message);

    /**
     * @brief re-serialize a message from the serialization of one transport to the one of the other
     * @return false if it can not be parsed
     */
    bool convert(const google::protobuf::Message &prototype, const std::string &input, Serialization *from,
                 Serialization *to, std::string *output);

    std::string convertReply(const ControlMessageType &requestType, const uint16_t &requestedTelemetry, const std::string &reply);

    void reportError(const std::string &message);

    Type type;
    TransportSharedPtr robotTransport;
    TransportSharedPtr controllerTransport;
    float timeout;
    float longRequestTimeout;

    std::map<uint16_t, std::shared_ptr<google::protobuf::Message>> commandTypes;
    std::map<uint16_t, std::shared_ptr<google::protobuf::Message>> telemetryTypes;

    std::atomic<uint64_t> forwarded;
    std::atomic<uint64_t> errors;
    std::map<std::string, uint64_t> reportedErrors;
};

}  // namespace robot_remote_control
