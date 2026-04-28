#include <cstdio>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <limits>

#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#pragma once

// declarations:

class controller : public rclcpp::Node
{
public:
    // variables

    // constructor
    controller();

private:
    // PARAMETERS
    

    
    /* ---- Diagnostics ---- */
    

    /* ---- Internal State for Diagnostics ---- */
    

    // variables

    sensor_msgs::msg::LaserScan scan_msg;

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

    // publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_speed_pub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gap_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr oh_no_im_stuck_publisher;

    // methods
    

    visualization_msgs::msg::Marker create_marker(double x, double y, double r, double g, double b, double size_factor, int id);

    visualization_msgs::msg::Marker create_line_marker(double x1, double y1, double x2, double y2, double r, double g, double b, double width, int id);
};
