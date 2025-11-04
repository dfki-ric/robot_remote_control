#include "ExtendedControlledRobot.hpp"


namespace robot_remote_control {

ExtendedControlledRobot::ExtendedControlledRobot(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport):
    ControlledRobot(commandTransport, telemetryTransport) {


        newControlMessageCommandBuffer = std::make_unique<CommandBuffer<myrobot::NewControlMessage>>(10);
        registerCommandType(NEW_CONTROL_MESSAGE, newControlMessageCommandBuffer.get());

        registerTelemetryType<myrobot::NewTelemetryMessage>(NEW_TELEMETRY_MESSAGE);
    }




}  // namespace robot_remote_control
