#include "TelemetryBuffer.hpp"


namespace robot_remote_control {


    TelemetryBuffer::TelemetryBuffer() {
        // create vector of shared ptr
        lock();
        //just pre-set sizes to minimize resize calls in registerType
        get_ref().resize(TELEMETRY_MESSAGE_TYPES_NUMBER);
        unlock();

        converters.resize(TELEMETRY_MESSAGE_TYPES_NUMBER);

    }

    TelemetryBuffer::~TelemetryBuffer() {
    }


    std::string TelemetryBuffer::peekSerialized(const uint16_t &type) {
        std::string buf("");
        lock();

        if (type != NO_TELEMETRY_DATA || type != TELEMETRY_MESSAGE_TYPES_NUMBER) {
            buf = converters[type]->get();
        }

        unlock();
        return buf;
    }

}  // namespace robot_remote_control
