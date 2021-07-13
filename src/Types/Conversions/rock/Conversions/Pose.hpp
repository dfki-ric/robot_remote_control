#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/Pose.hpp>

namespace robot_remote_control {
namespace RockConversion {

    // inline static void convert(const Poses &rrc_type, base::Trajectory* rock_type) {
    //     (*rock_type)[0] = rrc_type.x();
    //     (*rock_type)[1] = rrc_type.y();
    //     (*rock_type)[2] = rrc_type.z();
    // }

    inline static void convert(const base::Pose2D &rock_type, Pose *rrc_type) {
        rrc_type->mutable_position()->set_x(rock_type.position[0]);
        rrc_type->mutable_position()->set_y(rock_type.position[1]);
        rrc_type->set_orientation2d(rock_type.orientation);
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
