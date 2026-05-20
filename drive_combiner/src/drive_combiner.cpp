#include <algorithm>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;

struct TTCResult
{
    float min_ttc;
    float min_distance;
};

class DriveCombiner : public rclcpp::Node
{
public:
    DriveCombiner() : Node("drive_combiner"), current_speed_(0.0), reactive_mode_active_(false)
    {
        this->declare_parameter<double>("ttc_threshold", 1.0);
        this->declare_parameter<double>("ttc_cone_width_deg", 10.0);

        // subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DriveCombiner::scan_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DriveCombiner::odom_callback, this, _1));

        drive_pp_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_pp", 10, std::bind(&DriveCombiner::drive_pp_callback, this, _1));

        drive_re_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_re", 10, std::bind(&DriveCombiner::drive_re_callback, this, _1));

        // publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/drive_combiner/mode_marker", 10);
        cone_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/drive_combiner/cone_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Drive Combiner Node Started. Threshold ttc: %f, Cone Width: %f deg",
                    this->get_parameter("ttc_threshold").as_double(),
                    this->get_parameter("ttc_cone_width_deg").as_double());
    }

private:
    double current_speed_;
    bool reactive_mode_active_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pp_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_re_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cone_pub_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_speed_ = msg->twist.twist.linear.x;
    }

    TTCResult calc_min_ttc(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        double angle_increment = msg->angle_increment;
        double angle_min = msg->angle_min;
        std::vector<float> ranges = msg->ranges;

        // fetch and convert to radians
        double cone_width_deg = this->get_parameter("ttc_cone_width_deg").as_double();
        double fov_limit = (cone_width_deg / 2.0) * (M_PI / 180.0);

        int num_lasers = ranges.size();
        float min_TTC = std::numeric_limits<float>::infinity();
        float min_distance = std::numeric_limits<float>::infinity();

        for (int i = 0; i < num_lasers; i++)
        {
            double angle = i * angle_increment + angle_min;

            if (angle < -fov_limit || angle > fov_limit)
                continue;

            if (std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            double projection = std::max(0.0, current_speed_ * std::cos(angle));
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
        marker.header.frame_id = "ego_racecar/base_link";
        marker.header.stamp = stamp;
        marker.ns = "drive_mode";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.5;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;

        if (reactive_mode_active_)
        {
            // yellow
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else
        {
            // green
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }

        marker_pub_->publish(marker);
    }

    void publish_cone_marker(const std_msgs::msg::Header &header)
    {
        double cone_width_deg = this->get_parameter("ttc_cone_width_deg").as_double();
        double fov_limit = (cone_width_deg / 2.0) * (M_PI / 180.0);

        visualization_msgs::msg::Marker cone;
        cone.header = header;
        cone.ns = "ttc_cone";
        cone.id = 1;

        cone.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        cone.action = visualization_msgs::msg::Marker::ADD;

        cone.scale.x = 1.0;
        cone.scale.y = 1.0;
        cone.scale.z = 1.0;

        double visual_length = 3.0;

        // slightly elevate the Z-axis to prevent it from clipping into the floor
        double z_offset = 0.05;

        geometry_msgs::msg::Point origin, left_pt, right_pt;
        origin.x = 0.0;
        origin.y = 0.0;
        origin.z = z_offset;

        left_pt.x = visual_length * std::cos(fov_limit);
        left_pt.y = visual_length * std::sin(fov_limit);
        left_pt.z = z_offset;

        right_pt.x = visual_length * std::cos(-fov_limit);
        right_pt.y = visual_length * std::sin(-fov_limit);
        right_pt.z = z_offset;

        cone.points.push_back(origin);
        cone.points.push_back(left_pt);
        cone.points.push_back(right_pt);

        std_msgs::msg::ColorRGBA c;
        c.a = 0.5;
        if (reactive_mode_active_)
        {
            // yellow
            c.r = 1.0;
            c.g = 1.0;
            c.b = 0.0;
        }
        else
        {
            // green
            c.r = 0.0;
            c.g = 1.0;
            c.b = 0.0;
        }

        // set global color just in case
        cone.color = c;

        // assign the color to EVERY vertex of the triangle individually
        cone.colors.push_back(c);
        cone.colors.push_back(c);
        cone.colors.push_back(c);

        cone_pub_->publish(cone);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        double ttc_threshold = this->get_parameter("ttc_threshold").as_double();
        double min_ttc = calc_min_ttc(msg).min_ttc;

        if (min_ttc <= ttc_threshold)
        {
            if (!reactive_mode_active_)
            {
                RCLCPP_WARN(this->get_logger(), "SWITCH -> REACTIVE (TTC: %.2f < %.2f)", min_ttc, ttc_threshold);
                reactive_mode_active_ = true;
            }
        }
        else
        {
            if (reactive_mode_active_)
            {
                RCLCPP_WARN(this->get_logger(), "SWITCH -> PURE_PURSUIT (TTC: %.2f > %.2f)", min_ttc, ttc_threshold);
                reactive_mode_active_ = false;
            }
        }

        publish_mode_marker(msg->header.stamp);
        publish_cone_marker(msg->header);
    }

    void drive_pp_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
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