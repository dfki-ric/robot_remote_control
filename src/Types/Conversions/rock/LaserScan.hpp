#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <string>
#include <base/samples/LaserScan.hpp>
#include "Time.hpp"
#include <cinttypes>


namespace robot_remote_control {
namespace RockConversion {

    // Convert a base::samples::LaserScan to a robot_remote_control::LaserScan
    inline static void convert(base::samples::LaserScan rock_type, LaserScan *rrc_type) {
        
        // convert the time 
        // The ranges, min range, max range data are in millimeters but the RRC->ROS2 laserscan datatype uses meters so we need to convert them from mm to m
        convert(rock_type.time, rrc_type->mutable_header()->mutable_timestamp());

        rrc_type->set_start_angle(rock_type.start_angle);
        rrc_type->set_horizontal_angle_resolution(rock_type.angular_resolution);
        rrc_type->set_vertical_angle_resolution(0); // Assuming no vertical lines for horizontal laser scans
        rrc_type->set_scan_speed(rock_type.speed);

        rrc_type->set_range_min(rock_type.minRange / 1000.0); // Convert from mm to m
        rrc_type->set_range_max(rock_type.maxRange / 1000.0); // Convert from mm to m

        rrc_type->mutable_ranges()->Reserve(rock_type.ranges.size());

        rrc_type->set_vertical_size(0); // Assuming no vertical lines for horizontal laser scans
        rrc_type->set_horizontal_size(1); // Assuming no horizontal lines for vertical laser

        for (const auto& range : rock_type.ranges) {
            rrc_type->add_ranges(range / 1000.0); // Convert from mm to m
        }
    }

    inline static void convert(const LaserScan& rrc_type, base::samples::LaserScan *rock_type) {

        // The range data from ROS2->RRC Laserscan type is in meters, so we need to convert them from m to mm to visualize them correctly in the base::samples::LaserScan in ROCK
        convert(rrc_type.header().timestamp(), &(rock_type->time));

        rock_type->start_angle = rrc_type.start_angle();
        rock_type->angular_resolution = rrc_type.horizontal_angle_resolution();
        rock_type->speed = rrc_type.scan_speed();

        rock_type->minRange = static_cast<uint32_t>(rrc_type.range_min() * 1000.0); // Convert from m to mm
        rock_type->maxRange = static_cast<uint32_t>(rrc_type.range_max() * 1000.0); // Convert from m to mm

        rock_type->ranges.clear();
        rock_type->ranges.reserve(rrc_type.ranges_size());

        for (int i = 0; i < rrc_type.ranges_size(); ++i) {
            float range = rrc_type.ranges(i);
            rock_type->ranges.push_back(static_cast<uint32_t>(range * 1000.0)); // Convert from m to mm
        }
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
