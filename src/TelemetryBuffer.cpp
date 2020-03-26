#include "TelemetryBuffer.hpp"


namespace robot_remote_control {


    TelemetryBuffer::TelemetryBuffer(const size_t &size) {
        // create vector of shared ptr
        lock();
        //just pre-set sizes to minimize resize calls in registerType
        get_ref().resize(TELEMETRY_MESSAGE_TYPES_NUMBER);
        unlock();

        converters.resize(TELEMETRY_MESSAGE_TYPES_NUMBER);

        // fill shared pointers with objects

        registerType<Pose>(CURRENT_POSE, size);
        registerType<JointState>(JOINT_STATE, size);
        registerType<JointState>(CONTROLLABLE_JOINTS, size);
        registerType<SimpleActions>(SIMPLE_ACTIONS, size);
        registerType<ComplexActions>(COMPLEX_ACTIONS, size);
        registerType<RobotName>(ROBOT_NAME, size);
        registerType<RobotState>(ROBOT_STATE, size);
        registerType<LogMessage>(LOG_MESSAGE, size);
        registerType<VideoStreams>(VIDEO_STREAMS, size);
        registerType<SimpleSensors>(SIMPLE_SENSOR_DEFINITION, size);
        // simple sensors are stored in separate buffer when receiving, but sending requires this for requests
        registerType<SimpleSensor>(SIMPLE_SENSOR_VALUE, size);

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
