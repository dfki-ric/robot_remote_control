#include "ExtendedRobotController.hpp"


namespace robot_remote_control {

ExtendedRobotController::ExtendedRobotController(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport):
    RobotController(commandTransport, telemetryTransport) {

        //
        registerTelemetryType<myrobot::NewTelemetryMessage>(NEW_TELEMETRY_MESSAGE, 10);
}

}  // namespace robot_remote_control
