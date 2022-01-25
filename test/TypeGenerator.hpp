#include "../src/MessageTypes.hpp"

#include <cstdlib>
#include <string>

namespace robot_remote_control {

class TypeGenerator{
 public:

    static std::string genString() {
        const int len = (std::rand() / static_cast<double>(RAND_MAX)) * 30;
        static const char *chars = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
        std::string gen;
        gen.reserve(len);
        char* cur = const_cast<char*>(gen.data());
        char* end = cur+len;
        while (cur != end) {
            *cur = chars[std::rand() % (sizeof(chars) - 1)];
            ++cur;
        }
        return gen;
    }

    static Acceleration genAcceleration() {
        Acceleration data;
        *data.mutable_angular() = genVector3();
        *data.mutable_linear() = genVector3();
        return data;
    }

    static ComplexAction genComplexAction() {
        ComplexAction data;
        data.set_name(genString());
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
            *data.add_name() = genString();
            data.add_effort(std::rand());
            data.add_position(std::rand());
            data.add_velocity(std::rand());
        }
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
            *data.add_poses() = genPose();
        }
        return data;
    }

    static Position genPosition() {
        Position data;
        data.set_x(std::rand());
        data.set_y(std::rand());
        data.set_z(std::rand());
        return data;
    }

    static RobotName genRobotName() {
        RobotName data;
        data.set_value(genString());
        return data;
    }

    // static RobotState genRobotState() {
    //     RobotState data;
    //     data.set_state(std::to_string(std::rand()));
    //     return data;
    // }

    static SimpleActionDef genSimpleActionDef() {
        SimpleActionDef data;
        data.set_type(TRIGGER);
        data.set_max_state(std::rand());
        return data;
    }

    static SimpleAction genSimpleAction() {
        SimpleAction data;
        data.set_name(genString());
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

    static Transform genTransform() {
        Transform data;
        *data.mutable_transform() = genPose();
        data.set_from(genString());
        data.set_to(genString());
        return data;
    }

    static Twist genTwist() {
        Twist data;
        *data.mutable_angular() = genVector3();
        *data.mutable_linear() = genVector3();
        return data;
    }

    static Vector3 genVector3() {
        Vector3 data;
        data.set_x(std::rand());
        data.set_y(std::rand());
        data.set_z(std::rand());
        return data;
    }

    static VideoStream genVideoStream() {
        VideoStream data;
        *data.mutable_camerapose() = genPose();
        data.set_url(genString());
        return data;
    }

    static VideoStreams genVideoStreams() {
        VideoStreams data;
        for (int i = 0; i < 10; ++i) {
            *data.add_stream() = genVideoStream();
        }
        return data;
    }

    static Wrench genWrench() {
        Wrench data;
        *data.mutable_header() = genHeader();
        *data.mutable_force() = genVector3();
        *data.mutable_torque() = genVector3();
        return data;
    }

    static WrenchState genWrenchState() {
        WrenchState data;
        for (int values = 0; values < 10; ++values) {
            *data.add_wrenches() = genWrench();
        }
        return data;
    }

    static TimeStamp genTimeStamp() {
        TimeStamp data;
        data.set_secs(std::rand());
        data.set_nsecs(std::rand());
        return data;
    }

    static Header genHeader() {
        Header data;
        *data.mutable_timestamp() = genTimeStamp();
        data.set_frame(genString());
        data.set_seq(std::rand());
        return data;
    }

    static RegionOfInterest genRegionOfInterest() {
        RegionOfInterest data;
        data.set_x_offset(std::rand());
        data.set_y_offset(std::rand());
        data.set_width(std::rand());
        data.set_height(std::rand());
        data.set_do_rectify(std::rand() > (RAND_MAX / 2));
        return data;
    }

    static Image genImage() {
        Image data;
        *data.mutable_header() = genHeader();
        data.set_height(std::rand());
        data.set_width(std::rand());
        data.set_encoding(genString());
        data.set_is_bigendian(std::rand());
        data.set_step(std::rand());
        data.set_data(genString());
        return data;
    }

    static ImageLayers genImageLayers() {
        ImageLayers data;
        *data.mutable_header() = genHeader();
        for (int values = 0; values < 10; ++values) {
            *data.add_layers() = genImage();
        }
        return data;
    }

    static CameraInfo genCameraInfo() {
        CameraInfo data;
        *data.mutable_header() = genHeader();
        data.set_width(std::rand());
        data.set_height(std::rand());
        data.set_distortion_model(genString());
        for (int values = 0; values < 10; ++values) {
            data.add_d(std::rand());
            data.add_k(std::rand());
            data.add_r(std::rand());
            data.add_p(std::rand());
        }
        data.set_binning_x(std::rand());
        data.set_binning_y(std::rand());
        *data.mutable_roi() = genRegionOfInterest();
        return data;
    }

    static CameraInformation genCameraInformation() {
        CameraInformation data;
        CameraInfo* cam;
        for (int values = 0; values < 10; ++values) {
            *data.add_camerainfo() = genCameraInfo();
        }
        return data;
    }

    static PoseWithCovariance genPoseWithCovariance() {
        PoseWithCovariance data;
        *data.mutable_pose() = genPose();
        for (int values = 0; values < 36; ++values) {
            data.add_covariance(std::rand());
        }
        return data;
    }

    static TwistWithCovariance genTwistWithCovariance() {
        TwistWithCovariance data;
        *data.mutable_twist() = genTwist();
        for (int values = 0; values < 36; ++values) {
            data.add_covariance(std::rand());
        }
        return data;
    }

    static Odometry genOdometry() {
        Odometry data;
        *data.mutable_header() = genHeader();
        data.set_child_frame_id(genString());
        *data.mutable_pose() = genPoseWithCovariance();
        *data.mutable_twist() = genTwistWithCovariance();
        return data;
    }



};

}  // namespace robot_remote_control
