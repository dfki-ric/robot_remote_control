#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "../Header.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const sensor_msgs::msg::LaserScan &laserscan, robot_remote_control::LaserScan* to) {
        convert(laserscan.header, to->mutable_header());

        to->set_start_angle(laserscan.angle_min); // assuming angle_min is the start angle

        float scan_time = laserscan.scan_time;
        float scan_speed = 0;

        if (scan_time !=0){
            scan_speed = (laserscan.angle_max - laserscan.angle_min)/scan_time; // one full scan distance/scantime
        }
        else{
            scan_speed = 0;
        }

        to->set_scan_speed(scan_speed);

        to->set_range_min(laserscan.range_min);
        to->set_range_max(laserscan.range_max);

        to->set_vertical_size(0); // Assuming no vertical lines for horizontal laser scans
        to->set_horizontal_size(1); // Assuming no horizontal lines for vertical laser scans
        to->set_vertical_angle_resolution(0); // Assuming no vertical angle resolution for horizontal laser scans
        to->set_horizontal_angle_resolution(laserscan.angle_increment);

        for (const auto& range : laserscan.ranges) {
            to->add_ranges(range);
        }

    }

    static void convert(const robot_remote_control::LaserScan &laserscan, sensor_msgs::msg::LaserScan* to) {
        convert(laserscan.header(), &to->header);


        to->angle_min = laserscan.start_angle();

        to->angle_max = laserscan.start_angle() + (laserscan.ranges_size() - 1) * laserscan.horizontal_angle_resolution();
        to->angle_increment = laserscan.horizontal_angle_resolution();

        float scan_time = 0;
        float scan_speed = laserscan.scan_speed();

        if (scan_speed !=0){
            scan_time = (to->angle_max - to->angle_min)/scan_speed; // one full scan distance/scantime
        }
        else{
            scan_time = 0;
        }

        to->scan_time = scan_time;
        to->range_min = laserscan.range_min();
        to->range_max = laserscan.range_max();

        to->ranges.clear();
        to->ranges.reserve(laserscan.ranges_size());
        for (int i = 0; i < laserscan.ranges_size(); ++i) {
            to->ranges.push_back(laserscan.ranges(i));
        }

    }

}  // namespace RosConversion
}  // namespace robot_remote_control
