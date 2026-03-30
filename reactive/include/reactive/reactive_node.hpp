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
        int SAFETYBUBBLE = 80; // 
        double NORMALDRIVESPEED = 1.0;


        float THRESHOLD = 0.0; // min distance to be considered gap
        size_t n_gaphits = 160; // Need at least 


        // variables
        double goal_angle = 0;
        std::vector<double> gap_angles;
        std::vector<std::vector<double>> all_gaps;

        sensor_msgs::msg::LaserScan scan_msg;
        
        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        

        // publishers
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_speed_pub;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gap_publisher;

        

        // methods
        void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
        //int main(int argc, char ** argv);

        void visualize_lidar_ranges();

        visualization_msgs::msg::Marker create_marker(double x, double y, double r, double g, double b, double size_factor, int id);

        visualization_msgs::msg::Marker create_line_marker(double x1, double y1, double x2, double y2, double r, double g, double b, double width, int id);




};

