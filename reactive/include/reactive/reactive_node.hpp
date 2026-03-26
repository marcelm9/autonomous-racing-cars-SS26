#include <cstdio>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <limits>

#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

#pragma once

// declarations:

class FollowGap : public rclcpp::Node 
{
    public:
        // variables

        // constructor
        FollowGap();

    private:
        // PARAMETERS
        int SAFETYBUBBLE = 50; 
        double NORMALDRIVESPEED = 0.1;

        // variables
        sensor_msgs::msg::LaserScan scan_msg;

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        

        // publishers
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_speed_pub;

        

        // methods
        void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
        //int main(int argc, char ** argv);
};

