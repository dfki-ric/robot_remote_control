#pragma once

#include <string>
#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <odometry/ContactState.hpp>
#include "Time.hpp"
#include "Eigen.hpp"

namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(const ContactPoint &rrc_type, odometry::BodyContactPoint* rock_type) {
        convert(rrc_type.position(), &rock_type->position);
        rock_type->contact = rrc_type.contact();
        rock_type->slip = rrc_type.slip();
        rock_type->groupId = rrc_type.groupid();
    }

    // inline static void convert(const odometry::BodyContactPoint &rock_type, ContactPoint* rrc_type) {
        
    // }

    inline static void convert(const ContactPoints &rrc_type, odometry::BodyContactState* rock_type) {
        convert(rrc_type.header().timestamp(), &rock_type->time);
        rock_type->points.clear();
        rock_type->points.reserve(rrc_type.contacts().size());
        for (auto& point : rrc_type.contacts()) {
            odometry::BodyContactPoint rock_contact_point;
            convert(point, &rock_contact_point);
            rock_type->points.push_back(rock_contact_point);
        }
    }

    // inline static void convert(const odometry::BodyContactState &rock_type, ContactPoints* rrc_type) {
        
    // }

}  // namespace RockConversion
}  // namespace robot_remote_control
