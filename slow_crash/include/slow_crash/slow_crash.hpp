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

class SlowCrash : public rclcpp::Node
{
public:
    
    // constructor
    SlowCrash();

private:
    // PARAMETERS
    // variables
    rclcpp::TimerBase::SharedPtr timer_;

    // The function the timer calls
    void timer_callback();
    
    // variables
    


    // subscribers


    // publishers

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_speed;


    // methods
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    // int main(int argc, char ** argv);

};
