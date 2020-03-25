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


        /**
         * @brief get the serializes buffer value, so the calling function does not need to know the datatype.
         * 
         * @param type 
         * @return std::string 
         */
        virtual std::string peekSerialized(const uint16_t &type);

};

}  // namespace robot_remote_control
