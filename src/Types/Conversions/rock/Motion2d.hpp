#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/commands/Motion2D.hpp>
// #include "Eigen.hpp"
// #include "Pose.hpp"
// #include "Time.hpp"

namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(const Twist &rrc_type, base::commands::Motion2D* rock_type) {
        double angle = 0;
        if (rrc_type.linear().x() == 0 && rrc_type.linear().y() != 0) {
            angle = M_PI/2.0;
        } else if (rrc_type.linear().y() == 0) {
            angle = 0;
        } else {
            angle = atan(rrc_type.linear().y()/rrc_type.linear().x());
        }
        // accounting for negativ translations
        if (rrc_type.linear().x() < 0) {
            angle += M_PI;
        }

        rock_type->translation = sqrt(pow(rrc_type.linear().x(), 2) + pow(rrc_type.linear().y(), 2));
        rock_type->rotation = rrc_type.angular().z();
        rock_type->heading.rad = angle;
    }
}  // namespace RockConversion
}  // namespace robot_remote_control

