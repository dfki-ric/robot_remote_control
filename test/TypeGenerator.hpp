#include "../src/MessageTypes.hpp"

#include <cstdlib>

namespace robot_remote_control {

class TypeGenerator{
 public:
    static Position genPosition() {
        Position data;
        data.set_x(std::rand());
        data.set_y(std::rand());
        data.set_z(std::rand());
        return data;
    }


    static Orientation genOrentation() {
        Orientation data;
        data.set_x(std::rand());
        data.set_y(std::rand());
        data.set_z(std::rand());
        data.set_w(std::rand());
        return data;
    }

    static Pose genPose() {
        Pose data;
        *data.mutable_position() = genPosition();
        *data.mutable_orientation() = genOrentation();
        return data;
    }

    static Poses genPoses() {
        Poses data;
        for (int values = 0; values < 10; ++values) {
            *data.add_pose() = genPose();
        }
        return data;
    }

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

    static Acceleration genAcceleration() {
        Acceleration data;
        *data.mutable_angular() = genVector3();
        *data.mutable_linear() = genVector3();
        return data;
    }

    static GoTo genGoTo() {
        GoTo data;
        data.set_max_forward_speed(std::rand());
        data.set_waypoint_max_forward_speed(std::rand());
        *data.mutable_waypoint_pose() = genPose();
        return data;
    }

    static JointState genJointState() {
        JointState data;
        for (int values = 0; values < 10; ++values) {
            *data.add_name() = std::to_string(values);
            data.add_effort(std::rand());
            data.add_position(std::rand());
            data.add_velocity(std::rand());
        }
        return data;
    }

    static Wrench genWrench() {
        Wrench data;
        *data.mutable_force() = genVector3();
        *data.mutable_torque() = genVector3();
        return data;
    }

    static WrenchState genWrenchState() {
        WrenchState data;
        for (int values = 0; values < 10; ++values) {
            *data.add_frame() = std::to_string(values);
            *data.add_wrenches() = genWrench();
        }
        return data;
    }

    static SimpleActionDef genSimpleActionDef() {
        SimpleActionDef data;
        data.set_type(TRIGGER);
        data.set_max_state(std::rand());
        return data;
    }

    static SimpleAction genSimpleAction() {
        SimpleAction data;
        data.set_name(std::to_string(std::rand()));
        data.set_state(std::rand());
        *data.mutable_type() = genSimpleActionDef();
        return data;
    }

    static SimpleActions genSimpleActions() {
        SimpleActions data;
        for (int i = 0; i < 10; ++i) {
            *data.add_actions() = genSimpleAction();
        }
        return data;
    }

    static ComplexAction genComplexAction() {
        ComplexAction data;
        data.set_name(std::to_string(std::rand()));
        data.set_type(POSE_LIST);
        for (int i = 0; i < 10; ++i) {
            *data.add_poses() = genPose();
        }
        return data;
    }

    static ComplexActions genComplexActions() {
        ComplexActions data;
        for (int i = 0; i < 10; ++i) {
            *data.add_actions() = genComplexAction();
        }
        return data;
    }

    static RobotName genRobotName() {
        RobotName data;
        data.set_value(std::to_string(std::rand()));
        return data;
    }

    // static RobotState genRobotState() {
    //     RobotState data;
    //     data.set_state(std::to_string(std::rand()));
    //     return data;
    // }

    static VideoStream genVideoStream() {
        VideoStream data;
        *data.mutable_camerapose() = genPose();
        data.set_url(std::to_string(std::rand()));
        return data;
    }

    static VideoStreams genVideoStreams() {
        VideoStreams data;
        for (int i = 0; i < 10; ++i) {
            *data.add_stream() = genVideoStream();
        }
        return data;
    }

    static Transform genTransform() {
        Transform data;
        *data.mutable_transform() = genPose();
        data.set_from(std::to_string(std::rand()));
        data.set_to(std::to_string(std::rand()));
        return data;
    }

};

}  // namespace robot_remote_control
