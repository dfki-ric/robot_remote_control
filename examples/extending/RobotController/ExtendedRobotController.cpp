#include "ExtendedRobotController.hpp"


namespace robot_remote_control {

ExtendedRobotController::ExtendedRobotController(TransportSharedPtr commandTransport, TransportSharedPtr telemetryTransport):
    RobotController(commandTransport, telemetryTransport, std::dynamic_pointer_cast<TelemetryBuffer>(std::shared_ptr<ExtendedTelemetryBuffer>(new ExtendedTelemetryBuffer(10)))) {
}


void ExtendedRobotController::setNewControlMessage(const myrobot::NewControlMessage & msg) {
    sendProtobufData(msg, NEW_CONTROL_MESSAGE);
}




}  // namespace robot_remote_control