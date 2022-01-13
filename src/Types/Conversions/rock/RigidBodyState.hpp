#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/samples/RigidBodyState.hpp>
#include "Eigen.hpp"
#include "Pose.hpp"
#include "Time.hpp"

namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(const IMU &rrc_type, base::samples::RigidBodyState* rock_type) {
        convert(rrc_type.orientation(), &(rock_type->orientation));
    }

    inline static void convert(const base::samples::RigidBodyState &rock_type, Pose *rrc_type) {
        convert(rock_type.time, rrc_type->mutable_timestamp());
        convert(rock_type.position, rrc_type->mutable_position());
        convert(rock_type.orientation, rrc_type->mutable_orientation());
    }

    inline static void convert(const Pose &rrc_type, base::samples::RigidBodyState *rock_type) {
        convert(rrc_type.timestamp(), &rock_type->time);
        convert(rrc_type.position(), &rock_type->position);
        convert(rrc_type.orientation(), &rock_type->orientation);
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
