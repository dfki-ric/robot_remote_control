#pragma once

#include <robot_remote_control/ControlledRobot.hpp>
#include "../ExtendedMessageTypes.hpp"

namespace robot_remote_control {

class ExtendedControlledRobot : public ControlledRobot {
 public:
    ExtendedControlledRobot(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport);

    bool getNewControlMessage(myrobot::NewControlMessage *command) {
        return newControlMessageCommand.read(command);
    }

    int setNewTelemetryMessage(const myrobot::NewTelemetryMessage& telemetry) {
        return sendTelemetry(telemetry, NEW_TELEMETRY_MESSAGE);
    }

 private:
    CommandBuffer<myrobot::NewControlMessage> newControlMessageCommand;
};

}  // namespace robot_remote_control
