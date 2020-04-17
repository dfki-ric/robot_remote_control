#include "../src/ExtendedMessageTypes.hpp"

#include <cstdlib>

using namespace robot_remote_control;
using namespace myrobot;

class TypeGenerator{
 public:

    static Vector3 genVector3() {
        Vector3 data;
        data.set_x(std::rand());
        data.set_y(std::rand());
        data.set_z(std::rand());
        return data;
    }

    static Twist genTwist() {
        Twist data;
        *data.mutable_angular() = genVector3();
        *data.mutable_linear() = genVector3();
        return data;
    }

    static NewControlMessage genNewControlMessage() {
        NewControlMessage data;
	data.set_sequence_no(std::rand());
	data.set_msg(std::to_string(std::rand()));
	return data;
    }

    static RobotName genRobotName() {
        RobotName data;
        data.set_value(std::to_string(std::rand()));
        return data;
    }

    static NewTelemetryMessage genNewTelemetryMessage() {
        NewTelemetryMessage data;
	data.set_sequence_no(std::rand());
	data.set_msg(std::to_string(std::rand()));
	return data;
    }
};
