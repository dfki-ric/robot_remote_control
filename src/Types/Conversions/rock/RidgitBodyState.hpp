#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/samples/RigidBodyState.hpp>
#include "Eigen.hpp"
#include "Pose.hpp"

namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(const IMU &rrc_type, base::samples::RigidBodyState* rock_type) {
        convert(rrc_type.orientation(), &(rock_type->orientation));
    }

    // inline static void convert(const base::samples::RigidBodyState &rock_type, IMU *rrc_type) {
    // }

}  // namespace RockConversion
}  // namespace robot_remote_control
