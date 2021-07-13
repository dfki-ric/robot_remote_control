#pragma once

#include <vector>
#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <trajectory_follower/SubTrajectory.hpp>
#include "Pose.hpp"

namespace robot_remote_control {
namespace RockConversion {

    // inline static void convert(const Poses &rrc_type, base::Trajectory* rock_type) {
        // no easy conversion (data missing)
    // }

    inline static void convert(const std::vector<trajectory_follower::SubTrajectory> &rock_type, Poses *rrc_type) {
        for (auto &sub : rock_type) {
            convert(sub.goalPose, rrc_type->add_pose());
        }
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
