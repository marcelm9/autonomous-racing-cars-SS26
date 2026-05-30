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
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

struct TTCResult
{
    float min_ttc;
    float min_distance;
};

class SpeedLimiter : public rclcpp::Node
{
public:
    SpeedLimiter() : Node("speed_limiter"),
                     current_speed_(0.0),
                     is_limiting_(false),
                     last_min_dist_(std::numeric_limits<double>::infinity()),
                     has_goal_(false),
                     tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
                     tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        // Declare parameters
        this->declare_parameter<double>("ttc_threshold", 2.0); // Also acts as a time gap for safe speed
        this->declare_parameter<double>("ttc_cone_width_deg", 10.0);
        this->declare_parameter<std::string>("base_frame", "ego_racecar/base_link");
        this->declare_parameter<std::string>("laser_frame", "ego_racecar/laser");
        this->declare_parameter<double>("vehicle_width", 0.2);
        this->declare_parameter<double>("safety_margin", 0.1);
        this->declare_parameter<double>("ttc_lookahead_distance", 5.0);
        this->declare_parameter<double>("visual_path_length", 2.0);
        this->declare_parameter<double>("collision_distance_threshold", 0.5); // Min dist before complete stop
        this->declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("pp_input_topic", "/drive_pp");
        this->declare_parameter<std::string>("drive_output_topic", "/drive");
        this->declare_parameter<std::string>("goal_point_topic", "/raceline_pp/goal_point");
        this->declare_parameter<std::string>("path_topic", "/raceline/path");

        base_frame_ = this->get_parameter("base_frame").as_string();
        laser_frame_ = this->get_parameter("laser_frame").as_string();
        ttc_threshold_ = this->get_parameter("ttc_threshold").as_double();
        collision_dist_thresh_ = this->get_parameter("collision_distance_threshold").as_double();
        vehicle_width_ = this->get_parameter("vehicle_width").as_double();
        safety_margin_ = this->get_parameter("safety_margin").as_double();
        ttc_lookahead_distance_ = this->get_parameter("ttc_lookahead_distance").as_double();
        visual_path_length_ = this->get_parameter("visual_path_length").as_double();

        // subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_topic").as_string().c_str(), 10, std::bind(&SpeedLimiter::scan_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_topic").as_string().c_str(), 10, std::bind(&SpeedLimiter::odom_callback, this, _1));

        drive_pp_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            this->get_parameter("pp_input_topic").as_string().c_str(), 10, std::bind(&SpeedLimiter::drive_pp_callback, this, _1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            this->get_parameter("goal_point_topic").as_string().c_str(), 10, std::bind(&SpeedLimiter::goal_point_callback, this, _1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            this->get_parameter("path_topic").as_string().c_str(), 1, std::bind(&SpeedLimiter::path_callback, this, _1));

        // publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->get_parameter("drive_output_topic").as_string().c_str(), 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/speed_limiter/mode_marker", 10);
        cone_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/speed_limiter/cone_marker", 10);

        // Register the dynamic parameter callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&SpeedLimiter::parameters_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Speed Limiter Node Started.");
    }

private:
    double current_speed_;
    bool is_limiting_;
    double last_min_dist_;

    nav_msgs::msg::Path::SharedPtr latest_path_;
    std::vector<geometry_msgs::msg::Point> transformed_path_points_;
    geometry_msgs::msg::PointStamped latest_goal_point_stamped_;
    bool has_goal_;

    std::string base_frame_;
    std::string laser_frame_;
    double ttc_threshold_;
    double collision_dist_thresh_;
    double vehicle_width_;
    double safety_margin_;
    double ttc_lookahead_distance_;
    double visual_path_length_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pp_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cone_pub_;

    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters)
        {
            if (param.get_name() == "ttc_threshold" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                ttc_threshold_ = param.as_double();
            }
            else if (param.get_name() == "collision_distance_threshold" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                collision_dist_thresh_ = param.as_double();
            }
            else if (param.get_name() == "vehicle_width" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                vehicle_width_ = param.as_double();
            }
            else if (param.get_name() == "safety_margin" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                safety_margin_ = param.as_double();
            }
            else if (param.get_name() == "ttc_lookahead_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                ttc_lookahead_distance_ = param.as_double();
            }
            else if (param.get_name() == "visual_path_length" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                visual_path_length_ = param.as_double();
            }
        }
        return result;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        latest_path_ = msg;
    }

    void goal_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        latest_goal_point_stamped_ = *msg;
        has_goal_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    }

    double distance_to_segment(double lx, double ly, const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
    {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double l2 = dx * dx + dy * dy;
        if (l2 < 1e-6)
            return std::hypot(lx - p1.x, ly - p1.y);
        double t = std::max(0.0, std::min(1.0, ((lx - p1.x) * dx + (ly - p1.y) * dy) / l2));
        return std::hypot(lx - (p1.x + t * dx), ly - (p1.y + t * dy));
    }

    TTCResult calc_min_ttc(const sensor_msgs::msg::LaserScan::SharedPtr msg, const geometry_msgs::msg::TransformStamped &laser_to_base_tf)
    {
        double angle_increment = msg->angle_increment;
        double angle_min = msg->angle_min;
        const std::vector<float> &ranges = msg->ranges;

        double safety_width = vehicle_width_ / 2.0 + safety_margin_;

        int num_lasers = ranges.size();
        float min_TTC = std::numeric_limits<float>::infinity();
        float min_distance = std::numeric_limits<float>::infinity();

        if (transformed_path_points_.size() < 2)
            return {min_TTC, min_distance};

        geometry_msgs::msg::Point pt_laser;
        geometry_msgs::msg::Point pt_base;
        pt_laser.z = 0.0;

        for (int i = 0; i < num_lasers; i++)
        {
            if (std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            double angle = i * angle_increment + angle_min;

            // 1. Get Cartesian coordinates in the raw Lidar frame
            pt_laser.x = ranges[i] * std::cos(angle);
            pt_laser.y = ranges[i] * std::sin(angle);

            // 2. Transform the point into base_frame_
            tf2::doTransform(pt_laser, pt_base, laser_to_base_tf);

            // Find if this laser point is near any segment of the raceline
            bool point_on_path = false;
            for (size_t j = 0; j < transformed_path_points_.size() - 1; ++j)
            {
                if (transformed_path_points_[j].x < -0.1 && transformed_path_points_[j + 1].x < -0.1)
                    continue;

                if (distance_to_segment(pt_base.x, pt_base.y, transformed_path_points_[j], transformed_path_points_[j + 1]) < safety_width)
                {
                    point_on_path = true;
                    break;
                }
            }

            if (!point_on_path)
                continue;

            // 3. Calculate TTC using the angle relative to base_link
            double base_angle = std::atan2(pt_base.y, pt_base.x);
            double dist_base = std::hypot(pt_base.x, pt_base.y);

            double effective_speed = std::max(0.1, current_speed_);
            double projection = std::max(0.0, effective_speed * std::cos(base_angle));

            float ttc = (projection <= 1e-6) ? std::numeric_limits<float>::infinity() : (dist_base / projection);

            if (ttc < min_TTC)
                min_TTC = ttc;
            if (dist_base < min_distance)
                min_distance = dist_base;
        }

        return {min_TTC, min_distance};
    }

    void publish_mode_marker(const builtin_interfaces::msg::Time &stamp)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_frame_;
        marker.header.stamp = stamp;
        marker.ns = "speed_limit_mode";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.z = 0.5;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;

        if (is_limiting_)
        {
            marker.color.r = 1.0; // Red for limiting
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.r = 0.0; // Green for clear
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        marker_pub_->publish(marker);
    }

    void publish_cone_marker(const std_msgs::msg::Header &header)
    {
        double safety_width = vehicle_width_ / 2.0 + safety_margin_;
        double visual_length = visual_path_length_;

        visualization_msgs::msg::Marker cone;
        cone.header = header;
        cone.header.frame_id = base_frame_;
        cone.ns = "ttc_cone";
        cone.id = 1;
        cone.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        cone.action = visualization_msgs::msg::Marker::ADD;
        cone.pose.orientation.w = 1.0;
        cone.scale.x = 1.0;
        cone.scale.y = 1.0;
        cone.scale.z = 1.0;

        if (transformed_path_points_.size() < 2)
            return;

        double accumulated_dist = 0.0;
        for (size_t i = 0; i < transformed_path_points_.size() - 1; ++i)
        {
            const auto &p1 = transformed_path_points_[i];
            const auto &p2 = transformed_path_points_[i + 1];

            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double len = std::hypot(dx, dy);

            accumulated_dist += len;
            if (accumulated_dist > visual_length)
                break;

            if (len < 1e-6)
                continue;

            double nx = -dy / len;
            double ny = dx / len;

            geometry_msgs::msg::Point p1_left, p1_right, p2_left, p2_right;
            p1_left.x = p1.x + nx * safety_width;
            p1_left.y = p1.y + ny * safety_width;
            p1_left.z = 0.05;

            p1_right.x = p1.x - nx * safety_width;
            p1_right.y = p1.y - ny * safety_width;
            p1_right.z = 0.05;

            p2_left.x = p2.x + nx * safety_width;
            p2_left.y = p2.y + ny * safety_width;
            p2_left.z = 0.05;

            p2_right.x = p2.x - nx * safety_width;
            p2_right.y = p2.y - ny * safety_width;
            p2_right.z = 0.05;

            // Two triangles for each segment
            cone.points.push_back(p1_left);
            cone.points.push_back(p1_right);
            cone.points.push_back(p2_left);

            cone.points.push_back(p1_right);
            cone.points.push_back(p2_right);
            cone.points.push_back(p2_left);
        }

        std_msgs::msg::ColorRGBA c;
        c.a = 0.5;
        if (is_limiting_)
        {
            c.r = 1.0;
            c.g = 0.0;
            c.b = 0.0;
        }
        else
        {
            c.r = 0.0;
            c.g = 1.0;
            c.b = 0.0;
        }

        cone.color = c;
        for (size_t i = 0; i < cone.points.size(); ++i)
        {
            cone.colors.push_back(c);
        }

        cone_pub_->publish(cone);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 1. Get transform from the actual Laser frame to base_frame_
        geometry_msgs::msg::TransformStamped laser_to_base_tf;
        try
        {
            laser_to_base_tf = tf_buffer_->lookupTransform(
                base_frame_, laser_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Laser->Base transform missing: %s", ex.what());
            return;
        }

        // <-- TRANSFORM AND FILTER RACELINE HERE -->
        transformed_path_points_.clear();
        if (latest_path_ && !latest_path_->poses.empty())
        {
            try
            {
                std::string target_frame = base_frame_;
                double ttc_lookahead = ttc_lookahead_distance_;

                geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
                    target_frame, latest_path_->header.frame_id, tf2::TimePointZero);

                size_t nearest_idx = 0;
                double min_dist_sq = std::numeric_limits<double>::max();

                geometry_msgs::msg::TransformStamped tf_inv = tf_buffer_->lookupTransform(
                    latest_path_->header.frame_id, base_frame_, tf2::TimePointZero);
                double car_x = tf_inv.transform.translation.x;
                double car_y = tf_inv.transform.translation.y;

                for (size_t i = 0; i < latest_path_->poses.size(); ++i)
                {
                    double dx = car_x - latest_path_->poses[i].pose.position.x;
                    double dy = car_y - latest_path_->poses[i].pose.position.y;
                    double dist_sq = dx * dx + dy * dy;
                    if (dist_sq < min_dist_sq)
                    {
                        min_dist_sq = dist_sq;
                        nearest_idx = i;
                    }
                }

                double accumulated_dist = 0.0;
                size_t current_idx = nearest_idx;
                size_t n_points = latest_path_->poses.size();

                for (size_t i = 0; i < n_points; ++i)
                {
                    size_t idx = (current_idx + i) % n_points;

                    geometry_msgs::msg::PointStamped ps_in, ps_out;
                    ps_in.header = latest_path_->header;
                    ps_in.point = latest_path_->poses[idx].pose.position;
                    tf2::doTransform(ps_in, ps_out, tf);

                    transformed_path_points_.push_back(ps_out.point);

                    if (i > 0)
                    {
                        size_t prev_idx = (current_idx + i - 1) % n_points;
                        double dx = latest_path_->poses[idx].pose.position.x - latest_path_->poses[prev_idx].pose.position.x;
                        double dy = latest_path_->poses[idx].pose.position.y - latest_path_->poses[prev_idx].pose.position.y;
                        accumulated_dist += std::hypot(dx, dy);
                    }

                    if (accumulated_dist > ttc_lookahead)
                        break;
                }
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Path transform failed: %s", ex.what());
            }
        }

        // Save minimum distance for our speed limiting logic
        TTCResult ttc_res = calc_min_ttc(msg, laser_to_base_tf);
        last_min_dist_ = ttc_res.min_distance;

        publish_mode_marker(msg->header.stamp);
        publish_cone_marker(msg->header);
    }

    void drive_pp_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        double original_speed = msg->drive.speed;
        double safe_speed = original_speed;

        // Apply dynamic speed limiting if an obstacle was detected in our path
        if (last_min_dist_ != std::numeric_limits<float>::infinity())
        {
            if (last_min_dist_ <= collision_dist_thresh_)
            {
                safe_speed = 0.0;
            }
            else
            {
                // Simple proportional ACC (Adaptive Cruise Control) formula.
                // We use ttc_threshold_ as our desired safety time gap here.
                safe_speed = (last_min_dist_ - collision_dist_thresh_) / ttc_threshold_;
            }
        }

        // Ensure we don't increase speed, just limit it if necessary
        if (original_speed > 0.0) 
        {
            msg->drive.speed = std::min(original_speed, safe_speed);
        }

        // Set visual indicator states
        is_limiting_ = (msg->drive.speed < original_speed);

        // Always publish the unmodified PP steering combined with the potentially modified speed
        drive_pub_->publish(*msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedLimiter>());
    rclcpp::shutdown();
    return 0;
}