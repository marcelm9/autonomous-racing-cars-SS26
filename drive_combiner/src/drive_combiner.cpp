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
        // Declare parameters
        this->declare_parameter<double>("ttc_threshold", 1.0);
        this->declare_parameter<double>("ttc_cone_width_deg", 10.0);
        this->declare_parameter<std::string>("base_frame", "ego_racecar/base_link"); // ego_racecar/base_link in sim
        this->declare_parameter<double>("switch_cooldown_seconds", 1.0);
        this->declare_parameter<std::string>("odom_topic");
        this->declare_parameter<std::string>("scan_topic");
        this->declare_parameter<std::string>("pp_input_topic");
        this->declare_parameter<std::string>("re_input_topic");
        this->declare_parameter<std::string>("drive_output_topic");

        // Initialize cooldown timer with current time
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

        // publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->get_parameter("drive_output_topic").as_string().c_str(), 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/drive_combiner/mode_marker", 10);
        cone_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/drive_combiner/cone_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Drive Combiner Node Started. Threshold ttc: %f, Cone Width: %f deg, Switch Cooldown: %f s",
                    this->get_parameter("ttc_threshold").as_double(),
                    this->get_parameter("ttc_cone_width_deg").as_double(),
                    this->get_parameter("switch_cooldown_seconds").as_double());
    }

private:
    double current_speed_;
    bool reactive_mode_active_;
    double last_pp_steering_ = 0.0;
    rclcpp::Time last_switch_time_;

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

    TTCResult calc_min_ttc(const sensor_msgs::msg::LaserScan::SharedPtr msg, double cone_center)
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

            // shift the FOV window by the cone_center
            if (angle < (cone_center - fov_limit) || angle > (cone_center + fov_limit))
                continue;

            if (std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            // project the speed along the intended path, not just straight ahead
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
        origin.x = 0.0;
        origin.y = 0.0;
        origin.z = z_offset;

        // offset the left and right points by the steering angle
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

        // Calculate time elapsed since our last mode switch
        rclcpp::Time current_time = this->now();
        double elapsed_seconds = (current_time - last_switch_time_).seconds();

        // if in PP, look where we are steering; if in Reactive, look straight ahead
        double cone_center = reactive_mode_active_ ? 0.0 : last_pp_steering_;

        double min_ttc = calc_min_ttc(msg, cone_center).min_ttc;

        bool want_reactive_mode = (min_ttc <= ttc_threshold);

        if (want_reactive_mode != reactive_mode_active_)
        {
            if (elapsed_seconds >= switch_cooldown_seconds)
            {
                reactive_mode_active_ = want_reactive_mode;
                last_switch_time_ = current_time; // reset cooldown clock

                if (reactive_mode_active_)
                {
                    RCLCPP_WARN(this->get_logger(), "SWITCH -> REACTIVE (TTC: %.2f < %.2f)", min_ttc, ttc_threshold);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "SWITCH -> PURE_PURSUIT (TTC: %.2f > %.2f)", min_ttc, ttc_threshold);
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