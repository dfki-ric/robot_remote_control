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

    void requestNewTelemetryMessage(myrobot::NewTelemetryMessage *msg) {
        requestTelemetry(NEW_TELEMETRY_MESSAGE, msg);
    }

    bool getNewTelemetryMessage(myrobot::NewTelemetryMessage *msg) {
        return getTelemetry(NEW_TELEMETRY_MESSAGE, msg);
    }
};

}  // namespace robot_remote_control
