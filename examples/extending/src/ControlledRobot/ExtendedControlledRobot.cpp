#include "ExtendedControlledRobot.hpp"


namespace robot_remote_control {

ExtendedControlledRobot::ExtendedControlledRobot(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport):
    ControlledRobot(commandTransport, telemetryTransport) {

        registerCommandType(NEW_CONTROL_MESSAGE, &newControlMessageCommand);

        registerTelemetryType<myrobot::NewTelemetryMessage>(NEW_TELEMETRY_MESSAGE, 10);
    }




}  // namespace robot_remote_control
