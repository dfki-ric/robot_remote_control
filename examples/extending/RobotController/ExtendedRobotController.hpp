#pragma once

#include <memory>

#include "RobotController.hpp"
#include "ExtendedTelemetryBuffer.hpp"

namespace robot_remote_control {

class ExtendedRobotController : public RobotController {
 public:
    ExtendedRobotController(TransportSharedPtr commandTransport,
                            TransportSharedPtr telemetryTransport = TransportSharedPtr());

    void setNewControlMessage(const myrobot::NewControlMessage & msg);

    void requestNewTelemetryMessage(myrobot::NewTelemetryMessage *msg) {
        requestTelemetry(NEW_TELEMETRY_MESSAGE, msg);
    }

    bool getNewTelemetryMessage(myrobot::NewTelemetryMessage *msg) {
        return getTelemetry(NEW_TELEMETRY_MESSAGE, msg);
    }
};

}  // namespace robot_remote_control
