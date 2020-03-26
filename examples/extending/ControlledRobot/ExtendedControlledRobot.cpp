#include "ExtendedControlledRobot.hpp"


namespace robot_remote_control {

ExtendedControlledRobot::ExtendedControlledRobot(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport):
    ControlledRobot(commandTransport, telemetryTransport, std::dynamic_pointer_cast<TelemetryBuffer>(std::shared_ptr<ExtendedTelemetryBuffer>(new ExtendedTelemetryBuffer()))) {
        registerCommandBuffer(NEW_CONTROL_MESSAGE, &newControlMessageCommand);
    }




}  // namespace robot_remote_control
