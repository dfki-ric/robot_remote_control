#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/samples/Twist.hpp>
#include "Eigen.hpp"
#include "Time.hpp"

//Conversion from and to base/samples/RigidBodyState can be found in RigidBodyState.hpp

namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(const Twist &rrc_type, base::samples::Twist* rock_type) {
        convert(rrc_type.linear(), &(rock_type->linear));
        convert(rrc_type.angular(), &(rock_type->angular));
        rock_type->frame_id = rrc_type.header().frame();
        convert(rrc_type.header(), &(rock_type->time));
    }

    inline static void convert(const base::samples::Twist &rock_type, Twist* rrc_type) {
        convert(rock_type.linear, rrc_type->mutable_linear());
        convert(rock_type.angular, rrc_type->mutable_angular());
        rrc_type->mutable_header()->set_frame(rock_type.frame_id);
        convert(rock_type.time, rrc_type->mutable_header());
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
