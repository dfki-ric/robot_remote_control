#include "ExtendedTelemetryBuffer.hpp"


namespace robot_remote_control {


    ExtendedTelemetryBuffer::ExtendedTelemetryBuffer(const size_t &size): TelemetryBuffer(size) {
        // create vector of shared ptr
        lock();
        get_ref().resize(EXTENDED_TELEMETRY_MESSAGE_TYPES_NUMBER);

        std::shared_ptr<RingBufferBase> newbuf;

        // fill shared pointers with objects
        newbuf = std::shared_ptr<RingBufferBase>(new RingBuffer<myrobot::NewTelemetryMessage>(size));
        get_ref()[NEW_TELEMETRY_MESSAGE] = newbuf;

        unlock();
    }

    ExtendedTelemetryBuffer::~ExtendedTelemetryBuffer() {
    }


    std::string ExtendedTelemetryBuffer::peekSerialized(const uint16_t &type) {
        printf("%s:%i\n",__PRETTY_FUNCTION__, __LINE__);
        std::string buf = TelemetryBuffer::peekSerialized(type);
        if (buf.size()) {
            return buf;
        }

        lock();

        switch (type) {
            case NEW_TELEMETRY_MESSAGE: {
                myrobot::NewTelemetryMessage data;
                fillBuffer(NEW_TELEMETRY_MESSAGE, &data, &buf);
                break;
            }
            case NO_TELEMETRY_DATA:
            break;
        }
        unlock();
        return buf;
    }

}  // namespace robot_remote_control
