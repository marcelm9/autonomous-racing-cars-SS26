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

class DriveCombiner : public rclcpp::Node
{
public:
    DriveCombiner() : Node("drive_combiner"),
                      current_speed_(0.0),
                      reactive_mode_active_(false),
                      has_goal_(false),
                      tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
                      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        // Declare parameters
        this->declare_parameter<double>("ttc_threshold", 10.0);
        this->declare_parameter<double>("ttc_cone_width_deg", 10.0);
        this->declare_parameter<std::string>("base_frame", "ego_racecar/base_link");
        this->declare_parameter<std::string>("laser_frame", "ego_racecar/laser");
        this->declare_parameter<double>("switch_cooldown_seconds", 1.0);
        this->declare_parameter<double>("raceline_distance_threshold", 0.5);
        this->declare_parameter<double>("vehicle_width", 0.2);
        this->declare_parameter<double>("safety_margin", 0.1);
        this->declare_parameter<double>("ttc_lookahead_distance", 5.0);
        this->declare_parameter<double>("visual_path_length", 2.0);
        this->declare_parameter<double>("collision_distance_threshold", 0.1);
        this->declare_parameter<std::string>("odom_topic");
        this->declare_parameter<std::string>("scan_topic");
        this->declare_parameter<std::string>("pp_input_topic");
        this->declare_parameter<std::string>("re_input_topic");
        this->declare_parameter<std::string>("drive_output_topic");
        this->declare_parameter<std::string>("goal_point_topic", "/raceline_pp/goal_point");
        this->declare_parameter<std::string>("path_topic", "/raceline/path");

        base_frame_ = this->get_parameter("base_frame").as_string();
        laser_frame_ = this->get_parameter("laser_frame").as_string();
        ttc_threshold_ = this->get_parameter("ttc_threshold").as_double();
        collision_dist_thresh_ = this->get_parameter("collision_distance_threshold").as_double();
        switch_cooldown_seconds_ = this->get_parameter("switch_cooldown_seconds").as_double();
        raceline_dist_thresh_ = this->get_parameter("raceline_distance_threshold").as_double();
        vehicle_width_ = this->get_parameter("vehicle_width").as_double();
        safety_margin_ = this->get_parameter("safety_margin").as_double();
        ttc_lookahead_distance_ = this->get_parameter("ttc_lookahead_distance").as_double();
        visual_path_length_ = this->get_parameter("visual_path_length").as_double();

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
        // subscribe to String /strategist/strategy
        strategy_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/strategist/strategy", 10, std::bind(&DriveCombiner::strategy_callback, this, _1));

        // publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->get_parameter("drive_output_topic").as_string().c_str(), 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/drive_combiner/mode_marker", 10);
        cone_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/drive_combiner/cone_marker", 10);

        // Register the dynamic parameter callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DriveCombiner::parameters_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Drive Combiner Node Started.");
    }

private:
    double current_speed_;

    nav_msgs::msg::Path::SharedPtr latest_path_;
    std::vector<geometry_msgs::msg::Point> transformed_path_points_;
    geometry_msgs::msg::PointStamped latest_goal_point_stamped_;
    bool has_goal_;

    bool reactive_mode_active_;
    double last_pp_steering_ = 0.0;
    double last_re_speed_ = 0.0;
    bool overtaking_allowed_ = true;
    bool old_overtake_strategy_ = true;
    rclcpp::Time last_switch_time_;

    std::string base_frame_;
    std::string laser_frame_;
    double ttc_threshold_;
    double collision_dist_thresh_;
    double switch_cooldown_seconds_;
    double raceline_dist_thresh_;
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
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_re_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr strategy_sub_;
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
                RCLCPP_INFO(this->get_logger(), "Parameter 'ttc_threshold' updated to: %.2f", ttc_threshold_);
            }
            else if (param.get_name() == "collision_distance_threshold" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                collision_dist_thresh_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter 'collision_distance_threshold' updated to: %.2f", collision_dist_thresh_);
            }
            else if (param.get_name() == "switch_cooldown_seconds" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                switch_cooldown_seconds_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter 'switch_cooldown_seconds' updated to: %.2f", switch_cooldown_seconds_);
            }
            else if (param.get_name() == "raceline_distance_threshold" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                raceline_dist_thresh_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter 'raceline_distance_threshold' updated to: %.2f", raceline_dist_thresh_);
            }
            else if (param.get_name() == "vehicle_width" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                vehicle_width_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter 'vehicle_width' updated to: %.2f", vehicle_width_);
            }
            else if (param.get_name() == "safety_margin" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                safety_margin_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter 'safety_margin' updated to: %.2f", safety_margin_);
            }
            else if (param.get_name() == "ttc_lookahead_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                ttc_lookahead_distance_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter 'ttc_lookahead_distance' updated to: %.2f", ttc_lookahead_distance_);
            }
            else if (param.get_name() == "visual_path_length" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                visual_path_length_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter 'visual_path_length' updated to: %.2f", visual_path_length_);
            }
            else if (param.get_name() == "base_frame" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                base_frame_ = param.as_string();
                RCLCPP_INFO(this->get_logger(), "Parameter 'base_frame' updated to: %s", base_frame_.c_str());
            }
            else if (param.get_name() == "laser_frame" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                laser_frame_ = param.as_string();
                RCLCPP_INFO(this->get_logger(), "Parameter 'laser_frame' updated to: %s", laser_frame_.c_str());
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

    double get_distance_to_raceline()
    {
        if (!latest_path_ || latest_path_->poses.empty())
        {
            return std::numeric_limits<double>::infinity();
        }

        double car_x = 0.0;
        double car_y = 0.0;

        try
        {
            geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
                latest_path_->header.frame_id, base_frame_, tf2::TimePointZero);
            car_x = tf.transform.translation.x;
            car_y = tf.transform.translation.y;
        }
        catch (const tf2::TransformException &ex)
        {
            return std::numeric_limits<double>::infinity();
        }

        double min_dist = std::numeric_limits<double>::infinity();

        for (const auto &pose_stamped : latest_path_->poses)
        {
            double dx = car_x - pose_stamped.pose.position.x;
            double dy = car_y - pose_stamped.pose.position.y;
            double dist = std::hypot(dx, dy);

            if (dist < min_dist)
            {
                min_dist = dist;
            }
        }
        return min_dist;
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
                // Because we are now in base_frame_, X is guaranteed to be forward
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

            // 3. Calculate TTC using the angle relative to base_link, not the Lidar
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
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        else
        {
            marker.color.r = 0.0;
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
        // MUST use the same frame as transformed_path_points_
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
        if (reactive_mode_active_)
        {
            c.r = 0.0;
            c.g = 0.0;
            c.b = 1.0;
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
        double ttc_threshold = ttc_threshold_;
        double collision_dist_thresh = collision_dist_thresh_;
        double switch_cooldown_seconds = switch_cooldown_seconds_;
        double raceline_dist_thresh = raceline_dist_thresh_;

        rclcpp::Time current_time = this->now();
        double elapsed_seconds = (current_time - last_switch_time_).seconds();

        // 1. Get transform from the actual Laser frame to base_frame_
        geometry_msgs::msg::TransformStamped laser_to_base_tf;
        try
        {
            // Override the message's frame_id with the TF tree's actual name
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
                // We now force the target frame to be base_frame_
                std::string target_frame = base_frame_;
                double ttc_lookahead = ttc_lookahead_distance_;

                geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
                    target_frame, latest_path_->header.frame_id, tf2::TimePointZero);

                // Find the nearest point index
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

                // Start from nearest_idx and take points ahead
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

        // Pass the laser-to-base transform into the calculation
        TTCResult ttc_res = calc_min_ttc(msg, laser_to_base_tf);
        double min_ttc = ttc_res.min_ttc;
        double min_dist = ttc_res.min_distance;

        bool want_reactive_mode = (min_ttc <= ttc_threshold) || (min_dist <= collision_dist_thresh);

        if (want_reactive_mode)
        {
            last_switch_time_ = current_time;

            if (!reactive_mode_active_)
            {
                reactive_mode_active_ = true;
                RCLCPP_INFO(this->get_logger(), "SWITCH -> BLUE/REACTIVE (TTC: %.2f, MinDist: %.2f)", min_ttc, min_dist);
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
                    RCLCPP_INFO(this->get_logger(), "SWITCH -> GREEN/PURE_PURSUIT (TTC: %.2f, MinDist: %.2f) | Path Dist: %.2f",
                                min_ttc, min_dist, current_dist_to_path);
                }
            }
        }

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                              "TTC: %.2f, MinDist: %.2f, Mode: %s",
                              min_ttc, min_dist, reactive_mode_active_ ? "REACTIVE" : "PURE_PURSUIT");

        publish_mode_marker(msg->header.stamp);
        publish_cone_marker(msg->header);
    }

    void drive_pp_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        last_pp_steering_ = msg->drive.steering_angle;

        if (!reactive_mode_active_)
        {
            drive_pub_->publish(*msg);
        }
        else if (reactive_mode_active_ && (!overtaking_allowed_))
        {
            // If overtaking is not allowed, we still want to publish the PP steering angle but with reactive speed.
            ackermann_msgs::msg::AckermannDriveStamped modified_msg = *msg;
            modified_msg.drive.speed = last_re_speed_;
            drive_pub_->publish(modified_msg);
        }
        
    }

    void drive_re_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        last_re_speed_ = msg->drive.speed;
        if (reactive_mode_active_ && overtaking_allowed_)
        {
            drive_pub_->publish(*msg);
        }
    }

    void strategy_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "aggressive")
        {
            if (old_overtake_strategy_ != true) {
                RCLCPP_INFO(this->get_logger(), "Overtaking strategy switched to AGGRESSIVE");
            }
            overtaking_allowed_ = true;
            old_overtake_strategy_ = true;
        }
        else if (msg->data == "defensive")
        {
            if (old_overtake_strategy_ != false) {
                RCLCPP_INFO(this->get_logger(), "Overtaking strategy switched to DEFENSIVE");
            }
            overtaking_allowed_ = false;
            old_overtake_strategy_ = false;
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