#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/samples/RigidBodyState.hpp>
#include "Eigen.hpp"
#include "Time.hpp"

//Conversion from and to base/samples/RigidBodyState can be found in RigidBodyState.hpp

namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(const base::samples::RigidBodyState &rock_type, Transform* rrc_type) {
        convert(rock_type.time, rrc_type->mutable_header());
        convert(rock_type, rrc_type->mutable_transform());
        rrc_type->set_from(rock_type.targetFrame);
        rrc_type->set_to(rock_type.sourceFrame);
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
