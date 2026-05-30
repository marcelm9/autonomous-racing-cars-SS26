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
#include "geometry_msgs/msg/point.hpp"
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
    double MAXDRIVESPEED = 6.0;
    double MINDRIVESPEED = 3.0;
    double MAXDISTANCE = 7.0; // Gap Distance at which max speed is reached
    double BRAKE_TUNE = 0.1; //0.075; // higher --> brake later

    // parameters for gap finding
    float MAX_THRESHOLD = 15; // Distance for the first gap search
    float MIN_THRESHOLD = 0.3;
    int DYN_THRESHOLD_INCREMENT = 15; // How many increments of threshold should be tried
    size_t N_GAP_HITS = 20; // Need at least
    float GAP_VIEW_ANGLE_PROCENT = 0.2; // How much should be considered from the ranges when looking for gaps

    double SIGMA_FRONT = 2.4;
    double SIGMA_SIDE = 0.2;
    double SIGMA_AMPLITUDE = 0.4;

    // parameters for the Disparity Extender
    int N_EXTEND = 100; // how many rays should be extended 
    double STEP_DEPTH = 0.5; // how deep the gap must be to be extended

    // parameters for continuity correction
    int FILTER_WINDOW_SIZE = 5;      // Median filter size
    double CONTINUITY_WEIGHT = 0.5;  // Penalty for steering jumps
    double TOP_PERCENTAGE = 0.1;     // Furthest rays to average per gap

    
    /* ---- Diagnostics ---- */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub;
    rclcpp::TimerBase::SharedPtr diag_timer;
    void diag_timer_callback();

    /* ---- Internal State for Diagnostics ---- */
    double diag_active_threshold = 0.0;
    double diag_dist_closest_right = 0.0;
    double diag_dist_closest_left = 0.0;
    double diag_dist_closest_front = 0.0;
    double diag_correction_left = 0.0;
    double diag_correction_right = 0.0;
    double diag_set_speed = 0.0;

    // variables
    double goal_angle = 0;
    double last_goal_angle = 0; // Previous steering target for continuity
    std::vector<double> gap_angles;
    std::vector<std::vector<double>> all_gaps;

    int bubble_index_start = 0;
    int bubble_index_end = 0;

    int closest_idx_right = -1;
    int closest_idx_left = -1;
    double closest_angle_right = 0.0;
    double closest_angle_left = 0.0;
    double closest_dist_right = 0.0;
    double closest_dist_left = 0.0;

    double goal_angle_before_width_clamp = 0.0;
    double goal_angle_before_steer_clamp = 0.0;

    bool oh_no_im_stuck = false;

    // Tracking for disparity origins (-1: Right Obstacle, 1: Left Obstacle, 0: None)
    std::vector<int> disparity_origins;

    sensor_msgs::msg::LaserScan scan_msg;
    nav_msgs::msg::Path path_msg;
    nav_msgs::msg::Odometry odom_msg;

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    // publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_speed_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gap_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr oh_no_im_stuck_publisher;

    // methods
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    void path_callback(const nav_msgs::msg::Path::ConstSharedPtr path_msg);
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
    
    // Dynamic Parameter Callback
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);

    void visualize_lidar_ranges();

    std::vector<float> preprocess_scan(const std::vector<float> &ranges);

    void disparity_extender(
        const std::vector<float> &ranges,
        int n_extend,
        float step_depth,
        std::vector<float> &out_extended_ranges,
        std::vector<int> &out_extended_ranges_mask
    );

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
