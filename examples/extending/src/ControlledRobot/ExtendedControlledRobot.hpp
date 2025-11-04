#pragma once

#include <robot_remote_control/ControlledRobot.hpp>
#include "../ExtendedMessageTypes.hpp"

namespace robot_remote_control {

class ExtendedControlledRobot : public ControlledRobot {
 public:
    ExtendedControlledRobot(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport);

    bool getNewControlMessage(myrobot::NewControlMessage *command, bool onlyNewest = true) {
        return newControlMessageCommandBuffer->read(command, onlyNewest);
    }

    int setNewTelemetryMessage(const myrobot::NewTelemetryMessage& telemetry, const ChannelId &channel = 0) {
        return sendTelemetry(telemetry, NEW_TELEMETRY_MESSAGE, false, channel);
    }

 private:
    std::unique_ptr<CommandBuffer<myrobot::NewControlMessage>> newControlMessageCommandBuffer;
};

}  // namespace robot_remote_control
