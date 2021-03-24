#include "TelemetryBuffer.hpp"


namespace robot_remote_control {


    TelemetryBuffer::TelemetryBuffer() {
        // create vector of shared ptr
        // just pre-set sizes to minimize resize calls in registerType
        lockedAccess()->resize(TELEMETRY_MESSAGE_TYPES_NUMBER);

        converters.resize(TELEMETRY_MESSAGE_TYPES_NUMBER);
    }

    TelemetryBuffer::~TelemetryBuffer() {
    }


    std::string TelemetryBuffer::peekSerialized(const uint16_t &type) {
        std::string buf("");
        if (type != NO_TELEMETRY_DATA || type != TELEMETRY_MESSAGE_TYPES_NUMBER) {
            buf = converters[type]->get();
        }

        return buf;
    }

}  // namespace robot_remote_control
