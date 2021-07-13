#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/Eigen.hpp>

namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(const Vector3 &rrc_type, base::Vector3d* rock_type) {
        (*rock_type)[0] = rrc_type.x();
        (*rock_type)[1] = rrc_type.y();
        (*rock_type)[2] = rrc_type.z();
    }

    inline static void convert(const base::Vector3d &rock_type, Vector3 *rrc_type) {
        rrc_type->set_x(rock_type[0]);
        rrc_type->set_y(rock_type[1]);
        rrc_type->set_z(rock_type[2]);
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
