#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/samples/IMUSensors.hpp>
#include "Eigen.hpp"
#include "Pose.hpp"

namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(const IMU &rrc_type, base::samples::IMUSensors* rock_type) {
        convert(rrc_type.acceleration(), &(rock_type->acc));
        convert(rrc_type.gyro(), &(rock_type->gyro));
        convert(rrc_type.mag(), &(rock_type->mag));
    }

    inline static void convert(const base::samples::IMUSensors &rock_type, IMU *rrc_type) {
        convert(rock_type.acc, rrc_type->mutable_acceleration());
        convert(rock_type.gyro, rrc_type->mutable_gyro());
        convert(rock_type.mag, rrc_type->mutable_mag());
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
