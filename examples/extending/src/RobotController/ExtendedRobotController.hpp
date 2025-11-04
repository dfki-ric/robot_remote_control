#pragma once

#include <memory>

#include <robot_remote_control/RobotController.hpp>
#include "../ExtendedMessageTypes.hpp"

namespace robot_remote_control {

class ExtendedRobotController : public RobotController {
 public:
    explicit ExtendedRobotController(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport = TransportSharedPtr());

    void setNewControlMessage(const myrobot::NewControlMessage & msg) {
        sendProtobufData(msg, NEW_CONTROL_MESSAGE);
    }

    void requestNewTelemetryMessage(myrobot::NewTelemetryMessage *msg, const ChannelId &channel = 0) {
        requestTelemetry(NEW_TELEMETRY_MESSAGE, msg, channel);
    }

    bool getNewTelemetryMessage(myrobot::NewTelemetryMessage *msg, bool onlyNewest = false, const ChannelId &channel = 0) {
        return getTelemetry(NEW_TELEMETRY_MESSAGE, msg, onlyNewest, channel);
    }
};

}  // namespace robot_remote_control
