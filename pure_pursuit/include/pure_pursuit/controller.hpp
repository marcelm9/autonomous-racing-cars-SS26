#include <cstdio>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <limits>

#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
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


    // subscribers
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub; 
    
   

    

    // publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_speed_pub;


    // methods
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    
};
