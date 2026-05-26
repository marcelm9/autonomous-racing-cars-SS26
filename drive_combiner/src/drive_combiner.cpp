#include <algorithm>
#include <cmath>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using std::placeholders::_1;

struct TTCResult
{
    float min_ttc;
    float min_distance;
};

class DriveCombiner : public rclcpp::Node
{
public:
    DriveCombiner() : Node("drive_combiner"), current_speed_(0.0), reactive_mode_active_(false), has_goal_(false)
    {
        // Declare parameters
        this->declare_parameter<double>("ttc_threshold", 1.0);
        this->declare_parameter<double>("ttc_cone_width_deg", 10.0);
        this->declare_parameter<std::string>("base_frame", "ego_racecar/base_link");
        this->declare_parameter<double>("switch_cooldown_seconds", 1.0);
        this->declare_parameter<double>("raceline_distance_threshold", 0.5);
        this->declare_parameter<std::string>("odom_topic");
        this->declare_parameter<std::string>("scan_topic");
        this->declare_parameter<std::string>("pp_input_topic");
        this->declare_parameter<std::string>("re_input_topic");
        this->declare_parameter<std::string>("drive_output_topic");
        this->declare_parameter<std::string>("goal_point_topic", "/raceline_pp/goal_point");
        this->declare_parameter<std::string>("path_topic", "/raceline/path");

        last_switch_time_ = this->now();

        // subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_topic").as_string().c_str(), 10, std::bind(&DriveCombiner::scan_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_topic").as_string().c_str(), 10, std::bind(&DriveCombiner::odom_callback, this, _1));

        drive_pp_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            this->get_parameter("pp_input_topic").as_string().c_str(), 10, std::bind(&DriveCombiner::drive_pp_callback, this, _1));

        drive_re_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            this->get_parameter("re_input_topic").as_string().c_str(), 10, std::bind(&DriveCombiner::drive_re_callback, this, _1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            this->get_parameter("goal_point_topic").as_string().c_str(), 10, std::bind(&DriveCombiner::goal_point_callback, this, _1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            this->get_parameter("path_topic").as_string().c_str(), 1, std::bind(&DriveCombiner::path_callback, this, _1));

        // publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->get_parameter("drive_output_topic").as_string().c_str(), 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/drive_combiner/mode_marker", 10);
        cone_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/drive_combiner/cone_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Drive Combiner Node Started.");
    }

private:
    double current_speed_;
    geometry_msgs::msg::Point current_position_;
    double current_yaw_ = 0.0;

    nav_msgs::msg::Path::SharedPtr latest_path_;
    geometry_msgs::msg::Point latest_goal_point_;
    bool has_goal_;

    bool reactive_mode_active_;
    double last_pp_steering_ = 0.0;
    rclcpp::Time last_switch_time_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pp_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_re_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cone_pub_;

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        latest_path_ = msg;
    }

    void goal_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Store the global goal point, calculate relative angle later in scan_callback
        latest_goal_point_ = msg->point;
        has_goal_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_speed_ = msg->twist.twist.linear.x;
        current_position_ = msg->pose.pose.position;

        // Extract Yaw from the Odometry Quaternion
        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }

    double get_distance_to_raceline()
    {
        if (!latest_path_ || latest_path_->poses.empty())
        {
            return std::numeric_limits<double>::infinity();
        }

        double min_dist = std::numeric_limits<double>::infinity();

        for (const auto &pose_stamped : latest_path_->poses)
        {
            double dx = current_position_.x - pose_stamped.pose.position.x;
            double dy = current_position_.y - pose_stamped.pose.position.y;
            double dist = std::hypot(dx, dy);

            if (dist < min_dist)
            {
                min_dist = dist;
            }
        }
        return min_dist;
    }

    TTCResult calc_min_ttc(const sensor_msgs::msg::LaserScan::SharedPtr msg, double cone_center)
    {
        double angle_increment = msg->angle_increment;
        double angle_min = msg->angle_min;
        std::vector<float> ranges = msg->ranges;

        double cone_width_deg = this->get_parameter("ttc_cone_width_deg").as_double();
        double fov_limit = (cone_width_deg / 2.0) * (M_PI / 180.0);

        int num_lasers = ranges.size();
        float min_TTC = std::numeric_limits<float>::infinity();
        float min_distance = std::numeric_limits<float>::infinity();

        for (int i = 0; i < num_lasers; i++)
        {
            double angle = i * angle_increment + angle_min;

            if (angle < (cone_center - fov_limit) || angle > (cone_center + fov_limit))
                continue;

            if (std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            double projection = std::max(0.0, current_speed_ * std::cos(angle - cone_center));
            float ttc = (projection <= 0.0) ? std::numeric_limits<float>::infinity() : (ranges[i] / projection);

            if (ttc < min_TTC)
                min_TTC = ttc;
            if (ranges[i] < min_distance)
                min_distance = ranges[i];
        }

        return {min_TTC, min_distance};
    }

    void publish_mode_marker(const builtin_interfaces::msg::Time &stamp)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = this->get_parameter("base_frame").as_string();
        marker.header.stamp = stamp;
        marker.ns = "drive_mode";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.z = 0.5;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;

        if (reactive_mode_active_)
        {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        marker_pub_->publish(marker);
    }

    void publish_cone_marker(const std_msgs::msg::Header &header, double cone_center)
    {
        double cone_width_deg = this->get_parameter("ttc_cone_width_deg").as_double();
        double fov_limit = (cone_width_deg / 2.0) * (M_PI / 180.0);

        visualization_msgs::msg::Marker cone;
        cone.header = header;
        cone.header.frame_id = this->get_parameter("base_frame").as_string();
        cone.ns = "ttc_cone";
        cone.id = 1;
        cone.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        cone.action = visualization_msgs::msg::Marker::ADD;
        cone.scale.x = 1.0;
        cone.scale.y = 1.0;
        cone.scale.z = 1.0;

        double visual_length = 3.0;
        double z_offset = 0.05;
        geometry_msgs::msg::Point origin, left_pt, right_pt;
        origin.z = z_offset;

        left_pt.x = visual_length * std::cos(cone_center + fov_limit);
        left_pt.y = visual_length * std::sin(cone_center + fov_limit);
        left_pt.z = z_offset;

        right_pt.x = visual_length * std::cos(cone_center - fov_limit);
        right_pt.y = visual_length * std::sin(cone_center - fov_limit);
        right_pt.z = z_offset;

        cone.points.push_back(origin);
        cone.points.push_back(left_pt);
        cone.points.push_back(right_pt);

        std_msgs::msg::ColorRGBA c;
        c.a = 0.5;
        if (reactive_mode_active_)
        {
            c.r = 1.0;
            c.g = 1.0;
            c.b = 0.0;
        }
        else
        {
            c.r = 0.0;
            c.g = 1.0;
            c.b = 0.0;
        }

        cone.color = c;
        cone.colors.push_back(c);
        cone.colors.push_back(c);
        cone.colors.push_back(c);

        cone_pub_->publish(cone);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        double ttc_threshold = this->get_parameter("ttc_threshold").as_double();
        double switch_cooldown_seconds = this->get_parameter("switch_cooldown_seconds").as_double();
        double raceline_dist_thresh = this->get_parameter("raceline_distance_threshold").as_double();

        rclcpp::Time current_time = this->now();
        double elapsed_seconds = (current_time - last_switch_time_).seconds();

        double cone_center = 0.0;

        // <-- CALCULATE LOCAL ANGLE HERE -->
        if (has_goal_)
        {
            // Vector from car to goal
            double dx = latest_goal_point_.x - current_position_.x;
            double dy = latest_goal_point_.y - current_position_.y;

            // Absolute angle to goal on the map
            double global_angle = std::atan2(dy, dx);

            // Local angle relative to the nose of the car
            cone_center = global_angle - current_yaw_;

            // Normalize to [-PI, PI] to prevent visual spinning/overflows
            while (cone_center > M_PI)
                cone_center -= 2.0 * M_PI;
            while (cone_center < -M_PI)
                cone_center += 2.0 * M_PI;
        }

        double min_ttc = calc_min_ttc(msg, cone_center).min_ttc;
        bool want_reactive_mode = (min_ttc <= ttc_threshold);

        if (want_reactive_mode)
        {
            last_switch_time_ = current_time;

            if (!reactive_mode_active_)
            {
                reactive_mode_active_ = true;
                RCLCPP_WARN(this->get_logger(), "SWITCH -> REACTIVE (TTC: %.2f <= %.2f)", min_ttc, ttc_threshold);
            }
        }
        else
        {
            if (reactive_mode_active_ && (elapsed_seconds >= switch_cooldown_seconds))
            {
                double current_dist_to_path = get_distance_to_raceline();

                if (current_dist_to_path <= raceline_dist_thresh)
                {
                    reactive_mode_active_ = false;
                    RCLCPP_WARN(this->get_logger(), "SWITCH -> PURE_PURSUIT (TTC: %.2f > %.2f) | Path Dist: %.2f <= %.2f",
                                min_ttc, ttc_threshold, current_dist_to_path, raceline_dist_thresh);
                }
                else
                {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "Reactive Mode Active: Waiting to get closer to raceline (Current Dist: %.2f > %.2f)",
                                         current_dist_to_path, raceline_dist_thresh);
                }
            }
        }

        publish_mode_marker(msg->header.stamp);
        publish_cone_marker(msg->header, cone_center);
    }

    void drive_pp_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        last_pp_steering_ = msg->drive.steering_angle;

        if (!reactive_mode_active_)
        {
            drive_pub_->publish(*msg);
        }
    }

    void drive_re_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        if (reactive_mode_active_)
        {
            drive_pub_->publish(*msg);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveCombiner>());
    rclcpp::shutdown();
    return 0;
}