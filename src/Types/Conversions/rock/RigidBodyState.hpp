#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/samples/RigidBodyState.hpp>
#include "Eigen.hpp"
#include "Pose.hpp"
#include "Time.hpp"
#include "Twist.hpp"

namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(const IMU &rrc_type, base::samples::RigidBodyState* rock_type) {
        convert(rrc_type.orientation(), &(rock_type->orientation));
    }

    inline static void convert(const base::samples::RigidBodyState &rock_type, Pose *rrc_type) {
        convert(rock_type.time, rrc_type->mutable_header()->mutable_timestamp());
        convert(rock_type.position, rrc_type->mutable_position());
        convert(rock_type.orientation, rrc_type->mutable_orientation());
    }

    inline static void convert(const Pose &rrc_type, base::samples::RigidBodyState *rock_type) {
        convert(rrc_type.header().timestamp(), &rock_type->time);
        convert(rrc_type.position(), &rock_type->position);
        convert(rrc_type.orientation(), &rock_type->orientation);
    }

    inline static void convert(const base::samples::RigidBodyState &rock_type, Twist *rrc_type) {
        convert(rock_type.time, rrc_type->mutable_header()->mutable_timestamp());
        convert(rock_type.velocity, rrc_type->mutable_linear());
        convert(rock_type.angular_velocity, rrc_type->mutable_angular());
    }

    inline static void convert(const Twist &rrc_type, base::samples::RigidBodyState *rock_type) {
        convert(rrc_type.header().timestamp(), &rock_type->time);
        convert(rrc_type.linear(), &rock_type->velocity);
        convert(rrc_type.angular(), &rock_type->angular_velocity);
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
