#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/samples/Frame.hpp>


namespace robot_remote_control {
namespace RockConversion {

    // inline static void convert(const IMU &rrc_type, base::samples::IMUSensors* rock_type) {
    //     convert(rrc_type.acceleration(), &(rock_type->acc));
    //     convert(rrc_type.gyro(), &(rock_type->gyro));
    //     convert(rrc_type.mag(), &(rock_type->mag));
    // }

    inline static void convert(const base::samples::frame::Frame &rock_type, Image *rrc_type) {
        rrc_type->set_height(rock_type.getHeight());
        rrc_type->set_width(rock_type.getWidth());
        rrc_type->set_data(reinterpret_cast<const char*>(rock_type.image.data()));
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
