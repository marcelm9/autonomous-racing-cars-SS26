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

class FollowGap : public rclcpp::Node
{
public:
    // variables

    // constructor
    FollowGap();

private:
    // PARAMETERS
    int SAFETYBUBBLE = 2; //
    double MAXDRIVESPEED = 3;
    double BRAKE_TUNE = 0.5; // higher --> brake later

    float MAX_THRESHOLD = 15; // Distance for the first gap search
    float MIN_THRESHOLD = 0.5;
    int DYN_THRESHOLD_INCREMENT = 15; // How many increments of threshold should be tried
    size_t n_gaphits = 30; // Need at least

    float GAP_VIEW_ANGLE_PROCENT = 0.2; // How much should be considered from the ranges when looking for gaps


    int TO_BUBBLE_MARGIN = 10; // minimum ray index count margin of the steering to the safety bubble

    /* ---- Diagnostics ---- */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub;
    rclcpp::TimerBase::SharedPtr diag_timer;
    void diag_timer_callback();

    /* ---- Internal State for Diagnostics ---- */
    double diag_active_threshold_ = 0.0;
    double diag_dist_closest_right_ = 0.0;
    double diag_dist_closest_left_ = 0.0;
    double diag_dist_closest_front_ = 0.0;
    double diag_correction_left_ = 0.0;
    double diag_correction_right_ = 0.0;
    double diag_set_speed_ = 0.0;

    // variables
    double goal_angle = 0;
    std::vector<double> gap_angles;
    std::vector<std::vector<double>> all_gaps;

    int bubble_index_start = 0;
    int bubble_index_end = 0;

    bool oh_no_im_stuck = false;

    sensor_msgs::msg::LaserScan scan_msg;

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

    // publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_speed_pub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gap_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr no_way_out_pub;

    // methods
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    // int main(int argc, char ** argv);

    void visualize_lidar_ranges();

    void find_gaps_by_threshold(
        const std::vector<float> &ranges,
        double threshold,
        double angle_min,
        double angle_increment,
        std::vector<std::vector<double>> &out_all_gaps,
        std::vector<double> &out_best_gap);

    visualization_msgs::msg::Marker create_marker(double x, double y, double r, double g, double b, double size_factor, int id);

    visualization_msgs::msg::Marker create_line_marker(double x1, double y1, double x2, double y2, double r, double g, double b, double width, int id);
};
