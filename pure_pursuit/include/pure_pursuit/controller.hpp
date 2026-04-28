#ifndef PURE_PURSUIT_CONTROLLER_HPP
#define PURE_PURSUIT_CONTROLLER_HPP

#include <cstdio>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <limits>

#include <chrono>

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
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    /* ---- Diagnostics ---- */
    

    /* ---- Internal State for Diagnostics ---- */
    

    // variables
    nav_msgs::msg::Path current_path;
    bool path_received_ = false;
    double L = 1.0;


    // subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub; 
    

    // publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_speed_pub;


    // methods
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

    // timer
    rclcpp::TimerBase::SharedPtr pure_persuit_timer;
    void pure_persuit_timer_callback();
    
};

#endif