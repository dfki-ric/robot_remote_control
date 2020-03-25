#include "ExtendedRobotController.hpp"


namespace robot_remote_control {

ExtendedRobotController::ExtendedRobotController(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport):
    RobotController(commandTransport, telemetryTransport, std::dynamic_pointer_cast<TelemetryBuffer>(std::shared_ptr<ExtendedTelemetryBuffer>(new ExtendedTelemetryBuffer(10)))) {
}

}  // namespace robot_remote_control
