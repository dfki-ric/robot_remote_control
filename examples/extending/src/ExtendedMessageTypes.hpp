#pragma once

#include "Types/myrobot.pb.h"

#include <robot_remote_control/MessageTypes.hpp>

namespace robot_remote_control {

    enum ExtendedControlMessageType{ NEW_CONTROL_MESSAGE = CONTROL_MESSAGE_TYPE_NUMBER, EXTENDED_CONTROL_MESSAGE_TYPE_NUMBER };

    enum ExtendedTelemetryMessageType{ NEW_TELEMETRY_MESSAGE = TELEMETRY_MESSAGE_TYPES_NUMBER, EXTENDED_TELEMETRY_MESSAGE_TYPES_NUMBER };


}  // end namespace robot_remote_control




