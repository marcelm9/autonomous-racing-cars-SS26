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
        // variables
        int SAFETYBUBBLE = 50; // 
        double NORMALDRIVESPEED = 2;

        double THRESHOLD = 1.1; // min distance to be considered gap
        int n_gabhits = 10; // Need at least 
        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        

        // publishers
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_speed_pub;

        

        // methods
        void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
        //int main(int argc, char ** argv);
};

