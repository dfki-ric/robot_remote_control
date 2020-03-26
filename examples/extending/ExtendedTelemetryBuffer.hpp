#pragma once


#include <memory>
#include <vector>
#include <string>

#include "TelemetryBuffer.hpp"
#include "ExtendedMessageTypes.hpp"


namespace robot_remote_control {

class ExtendedTelemetryBuffer: public TelemetryBuffer {
 public:
    explicit ExtendedTelemetryBuffer(const size_t &size = 1);

    ~ExtendedTelemetryBuffer();

    virtual std::string peekSerialized(const uint16_t &type);
};

}  // namespace robot_remote_control
