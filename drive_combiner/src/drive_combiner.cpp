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
    DriveCombiner()
        : Node("drive_combiner"),
          current_speed_(0.0),
          reactive_mode_active_(false),
          has_goal_(false),
          consecutive_reactive_frames_(0),
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
        this->declare_parameter<double>("min_forward_distance", 0.5);
        this->declare_parameter<int>("reactive_frames_threshold", 3);
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
        min_forward_distance_ = this->get_parameter("min_forward_distance").as_double();
        reactive_frames_threshold_ = this->get_parameter("reactive_frames_threshold").as_int();

        last_switch_time_ = this->now();

        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_topic").as_string(), 10,
            std::bind(&DriveCombiner::scan_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_topic").as_string(), 10,
            std::bind(&DriveCombiner::odom_callback, this, _1));

        drive_pp_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            this->get_parameter("pp_input_topic").as_string(), 10,
            std::bind(&DriveCombiner::drive_pp_callback, this, _1));

        drive_re_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            this->get_parameter("re_input_topic").as_string(), 10,
            std::bind(&DriveCombiner::drive_re_callback, this, _1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            this->get_parameter("goal_point_topic").as_string(), 10,
            std::bind(&DriveCombiner::goal_point_callback, this, _1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            this->get_parameter("path_topic").as_string(), 1,
            std::bind(&DriveCombiner::path_callback, this, _1));

        // Publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            this->get_parameter("drive_output_topic").as_string(), 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/drive_combiner/mode_marker", 10);
        cone_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/drive_combiner/cone_marker", 10);
        ttc_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/drive_combiner/ttc_marker", 10);

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DriveCombiner::parameters_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Drive Combiner Node Started.");
    }

private:
    // State
    double current_speed_;
    bool reactive_mode_active_;
    bool has_goal_;
    int consecutive_reactive_frames_;
    rclcpp::Time last_switch_time_;

    // Cached path data
    nav_msgs::msg::Path::SharedPtr latest_path_;
    std::vector<geometry_msgs::msg::Point> transformed_path_points_;
    geometry_msgs::msg::PointStamped latest_goal_point_stamped_;

    // Parameters
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
    double min_forward_distance_;
    int reactive_frames_threshold_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS handles
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pp_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_re_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cone_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ttc_marker_pub_;

    // -------------------------------------------------------------------------
    // Parameter callback
    // -------------------------------------------------------------------------
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        for (const auto &param : parameters)
        {
            const std::string &name = param.get_name();

            if (name == "ttc_threshold" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                ttc_threshold_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "ttc_threshold -> %.2f", ttc_threshold_);
            }
            else if (name == "collision_distance_threshold" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                collision_dist_thresh_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "collision_distance_threshold -> %.2f", collision_dist_thresh_);
            }
            else if (name == "switch_cooldown_seconds" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                switch_cooldown_seconds_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "switch_cooldown_seconds -> %.2f", switch_cooldown_seconds_);
            }
            else if (name == "raceline_distance_threshold" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                raceline_dist_thresh_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "raceline_distance_threshold -> %.2f", raceline_dist_thresh_);
            }
            else if (name == "vehicle_width" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                vehicle_width_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "vehicle_width -> %.2f", vehicle_width_);
            }
            else if (name == "safety_margin" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                safety_margin_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "safety_margin -> %.2f", safety_margin_);
            }
            else if (name == "ttc_lookahead_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                ttc_lookahead_distance_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "ttc_lookahead_distance -> %.2f", ttc_lookahead_distance_);
            }
            else if (name == "visual_path_length" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                visual_path_length_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "visual_path_length -> %.2f", visual_path_length_);
            }
            else if (name == "min_forward_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                min_forward_distance_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "min_forward_distance -> %.2f", min_forward_distance_);
            }
            else if (name == "reactive_frames_threshold" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                reactive_frames_threshold_ = param.as_int();
                RCLCPP_INFO(this->get_logger(), "reactive_frames_threshold -> %d", reactive_frames_threshold_);
            }
            else if (name == "base_frame" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                base_frame_ = param.as_string();
                RCLCPP_INFO(this->get_logger(), "base_frame -> %s", base_frame_.c_str());
            }
            else if (name == "laser_frame" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                laser_frame_ = param.as_string();
                RCLCPP_INFO(this->get_logger(), "laser_frame -> %s", laser_frame_.c_str());
            }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        return result;
    }

    // -------------------------------------------------------------------------
    // Subscribers
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    // Geometry helpers
    // -------------------------------------------------------------------------
    static double distance_to_segment(double px, double py,
                                      const geometry_msgs::msg::Point &a,
                                      const geometry_msgs::msg::Point &b)
    {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double l2 = dx * dx + dy * dy;
        if (l2 < 1e-6)
            return std::hypot(px - a.x, py - a.y);
        double t = std::clamp(((px - a.x) * dx + (py - a.y) * dy) / l2, 0.0, 1.0);
        return std::hypot(px - (a.x + t * dx), py - (a.y + t * dy));
    }

    // Returns the car's distance to the nearest raceline point (in the path's frame).
    double get_distance_to_raceline() const
    {
        if (!latest_path_ || latest_path_->poses.empty())
            return std::numeric_limits<double>::infinity();

        try
        {
            geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
                latest_path_->header.frame_id, base_frame_, tf2::TimePointZero);

            double car_x = tf.transform.translation.x;
            double car_y = tf.transform.translation.y;
            double min_dist = std::numeric_limits<double>::infinity();

            for (const auto &pose : latest_path_->poses)
            {
                double d = std::hypot(car_x - pose.pose.position.x,
                                      car_y - pose.pose.position.y);
                min_dist = std::min(min_dist, d);
            }
            return min_dist;
        }
        catch (const tf2::TransformException &)
        {
            return std::numeric_limits<double>::infinity();
        }
    }

    // -------------------------------------------------------------------------
    // TF helper: try stamped, fall back to latest
    // -------------------------------------------------------------------------
    geometry_msgs::msg::TransformStamped lookup_transform(
        const std::string &target, const std::string &source,
        const builtin_interfaces::msg::Time &stamp) const
    {
        try
        {
            return tf_buffer_->lookupTransform(target, source, stamp,
                                               rclcpp::Duration::from_seconds(0.05));
        }
        catch (...)
        {
            return tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
        }
    }

    // -------------------------------------------------------------------------
    // Build transformed_path_points_ in base_frame_
    //   - starts at the nearest path point to the car
    //   - accumulates arc-length in the transformed (base) frame
    //   - stops when ttc_lookahead_distance_ is reached
    // -------------------------------------------------------------------------
    void update_transformed_path(const builtin_interfaces::msg::Time &stamp)
    {
        transformed_path_points_.clear();

        if (!latest_path_ || latest_path_->poses.empty())
            return;

        try
        {
            // Transform: map frame → base_frame_
            auto tf = lookup_transform(base_frame_, latest_path_->header.frame_id, stamp);

            // Also need inverse to find the nearest point in map frame
            auto tf_inv = lookup_transform(latest_path_->header.frame_id, base_frame_, stamp);
            double car_x = tf_inv.transform.translation.x;
            double car_y = tf_inv.transform.translation.y;

            // Find nearest path index
            size_t nearest_idx = 0;
            double min_dist_sq = std::numeric_limits<double>::max();
            const size_t n = latest_path_->poses.size();

            for (size_t i = 0; i < n; ++i)
            {
                double dx = car_x - latest_path_->poses[i].pose.position.x;
                double dy = car_y - latest_path_->poses[i].pose.position.y;
                double d2 = dx * dx + dy * dy;
                if (d2 < min_dist_sq)
                {
                    min_dist_sq = d2;
                    nearest_idx = i;
                }
            }

            // Walk forward from nearest_idx, accumulating arc-length in base_frame_
            double accumulated_dist = 0.0;

            for (size_t i = 0; i < n; ++i)
            {
                size_t idx = (nearest_idx + i) % n;

                geometry_msgs::msg::PointStamped ps_in, ps_out;
                ps_in.header = latest_path_->header;
                ps_in.point = latest_path_->poses[idx].pose.position;
                tf2::doTransform(ps_in, ps_out, tf);

                transformed_path_points_.push_back(ps_out.point);

                // Accumulate arc-length using already-transformed points (base_frame_)
                // This avoids wrap-around jumps in the map frame.
                if (transformed_path_points_.size() >= 2)
                {
                    const auto &prev = transformed_path_points_[transformed_path_points_.size() - 2];
                    const auto &curr = transformed_path_points_.back();
                    accumulated_dist += std::hypot(curr.x - prev.x, curr.y - prev.y);
                }

                if (accumulated_dist > ttc_lookahead_distance_)
                    break;
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Path transform failed: %s", ex.what());
        }
    }

    // -------------------------------------------------------------------------
    // TTC calculation
    //   Key fixes vs. original:
    //     1. Skip any point that is not ahead of the car (pt_base.x < min_forward_distance_)
    //     2. Accumulate arc-length in base_frame_ (done in update_transformed_path)
    // -------------------------------------------------------------------------
    TTCResult calc_min_ttc(const sensor_msgs::msg::LaserScan::SharedPtr &msg,
                           const geometry_msgs::msg::TransformStamped &laser_to_base_tf) const
    {
        float min_ttc = std::numeric_limits<float>::infinity();
        float min_distance = std::numeric_limits<float>::infinity();

        if (transformed_path_points_.size() < 2)
            return {min_ttc, min_distance};

        const double safety_width = vehicle_width_ / 2.0 + safety_margin_;
        const double effective_speed = std::max(0.1, current_speed_);

        geometry_msgs::msg::Point pt_laser, pt_base;
        pt_laser.z = 0.0;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float r = msg->ranges[i];
            if (std::isnan(r) || std::isinf(r))
                continue;

            double angle = msg->angle_min + i * msg->angle_increment;
            pt_laser.x = r * std::cos(angle);
            pt_laser.y = r * std::sin(angle);

            tf2::doTransform(pt_laser, pt_base, laser_to_base_tf);

            // --- FIX 1: only consider points ahead of the car ---
            if (pt_base.x < min_forward_distance_)
                continue;

            // --- FIX 2: check if point lies within the raceline corridor ---
            bool point_on_path = false;
            for (size_t j = 0; j + 1 < transformed_path_points_.size(); ++j)
            {
                if (distance_to_segment(pt_base.x, pt_base.y,
                                        transformed_path_points_[j],
                                        transformed_path_points_[j + 1]) < safety_width)
                {
                    point_on_path = true;
                    break;
                }
            }

            if (!point_on_path)
                continue;

            // TTC uses the base_link angle so that only the closing speed matters
            double base_angle = std::atan2(pt_base.y, pt_base.x);
            double dist_base = std::hypot(pt_base.x, pt_base.y);
            double projection = std::max(0.0, effective_speed * std::cos(base_angle));

            float ttc = (projection <= 1e-6)
                            ? std::numeric_limits<float>::infinity()
                            : static_cast<float>(dist_base / projection);

            min_ttc = std::min(min_ttc, ttc);
            min_distance = std::min(min_distance, static_cast<float>(dist_base));
        }

        return {min_ttc, min_distance};
    }

    // -------------------------------------------------------------------------
    // Visualization helpers
    // -------------------------------------------------------------------------
    void publish_mode_marker(const builtin_interfaces::msg::Time &stamp)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = base_frame_;
        m.header.stamp = stamp;
        m.ns = "drive_mode";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.z = 0.5;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.2;
        m.color.a = 1.0;

        if (reactive_mode_active_)
        {
            m.color.b = 1.0;
        } // blue  = reactive
        else
        {
            m.color.g = 1.0;
        } // green = pure pursuit

        marker_pub_->publish(m);
    }

    void publish_cone_marker(const std_msgs::msg::Header &header)
    {
        if (transformed_path_points_.size() < 2)
            return;

        const double safety_width = vehicle_width_ / 2.0 + safety_margin_;

        visualization_msgs::msg::Marker cone;
        cone.header = header;
        cone.header.frame_id = base_frame_;
        cone.ns = "ttc_cone";
        cone.id = 1;
        cone.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        cone.action = visualization_msgs::msg::Marker::ADD;
        cone.pose.orientation.w = 1.0;
        cone.scale.x = cone.scale.y = cone.scale.z = 1.0;

        std_msgs::msg::ColorRGBA c;
        c.a = 0.5;
        if (reactive_mode_active_)
        {
            c.b = 1.0;
        }
        else
        {
            c.g = 1.0;
        }
        cone.color = c;

        double accumulated_dist = 0.0;
        for (size_t i = 0; i + 1 < transformed_path_points_.size(); ++i)
        {
            const auto &p1 = transformed_path_points_[i];
            const auto &p2 = transformed_path_points_[i + 1];

            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double len = std::hypot(dx, dy);

            if (len < 1e-6)
                continue;

            accumulated_dist += len;
            if (accumulated_dist > visual_path_length_)
                break;

            double nx = -dy / len;
            double ny = dx / len;

            auto make_pt = [](double x, double y)
            {
                geometry_msgs::msg::Point p;
                p.x = x;
                p.y = y;
                p.z = 0.05;
                return p;
            };

            auto p1l = make_pt(p1.x + nx * safety_width, p1.y + ny * safety_width);
            auto p1r = make_pt(p1.x - nx * safety_width, p1.y - ny * safety_width);
            auto p2l = make_pt(p2.x + nx * safety_width, p2.y + ny * safety_width);
            auto p2r = make_pt(p2.x - nx * safety_width, p2.y - ny * safety_width);

            cone.points.insert(cone.points.end(), {p1l, p1r, p2l, p1r, p2r, p2l});
            for (int k = 0; k < 6; ++k)
                cone.colors.push_back(c);
        }

        cone_pub_->publish(cone);
    }

    void publish_ttc_marker(const std_msgs::msg::Header &header, double ttc)
    {
        visualization_msgs::msg::Marker m;
        m.header = header;
        m.header.frame_id = base_frame_;
        m.ns = "ttc_text";
        m.id = 2;
        m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.z = 1.0;
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.4;
        m.color.a = m.color.r = m.color.g = m.color.b = 1.0;

        if (std::isinf(ttc))
        {
            m.text = "TTC: INF";
        }
        else
        {
            char buf[50];
            snprintf(buf, sizeof(buf), "TTC: %.2f s", ttc);
            m.text = buf;
            if (ttc <= ttc_threshold_) // red when dangerous
            {
                m.color.g = 0.0;
                m.color.b = 0.0;
            }
        }

        ttc_marker_pub_->publish(m);
    }

    // -------------------------------------------------------------------------
    // Main scan callback
    // -------------------------------------------------------------------------
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // --- Get laser→base transform ---
        geometry_msgs::msg::TransformStamped laser_to_base_tf;
        try
        {
            laser_to_base_tf = lookup_transform(base_frame_, laser_frame_, msg->header.stamp);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Laser->Base transform missing: %s", ex.what());
            return;
        }

        // --- Update the raceline corridor in base_frame_ ---
        update_transformed_path(msg->header.stamp);

        // --- Compute TTC ---
        TTCResult ttc_res = calc_min_ttc(msg, laser_to_base_tf);
        double min_ttc = ttc_res.min_ttc;
        double min_dist = ttc_res.min_distance;

        bool want_reactive = (min_ttc <= ttc_threshold_) ||
                             (min_dist <= collision_dist_thresh_);

        // --- Hysteresis: require N consecutive frames before switching to reactive ---
        if (want_reactive)
        {
            consecutive_reactive_frames_++;

            if (consecutive_reactive_frames_ >= reactive_frames_threshold_)
            {
                last_switch_time_ = this->now();

                if (!reactive_mode_active_)
                {
                    reactive_mode_active_ = true;
                    RCLCPP_INFO(this->get_logger(),
                                "SWITCH -> REACTIVE (TTC: %.2f, MinDist: %.2f)", min_ttc, min_dist);
                }
            }
        }
        else
        {
            // Reset frame counter immediately on any clean scan
            consecutive_reactive_frames_ = 0;

            if (reactive_mode_active_)
            {
                double elapsed = (this->now() - last_switch_time_).seconds();

                if (elapsed >= switch_cooldown_seconds_)
                {
                    double dist_to_path = get_distance_to_raceline();

                    if (dist_to_path <= raceline_dist_thresh_)
                    {
                        reactive_mode_active_ = false;
                        RCLCPP_INFO(this->get_logger(),
                                    "SWITCH -> PURE_PURSUIT (TTC: %.2f, MinDist: %.2f, PathDist: %.2f)",
                                    min_ttc, min_dist, dist_to_path);
                    }
                }
            }
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "TTC: %.2f | MinDist: %.2f | Frames: %d/%d | Mode: %s",
                             min_ttc, min_dist,
                             consecutive_reactive_frames_, reactive_frames_threshold_,
                             reactive_mode_active_ ? "REACTIVE" : "PURE_PURSUIT");

        // --- Publish visualizations ---
        auto now = this->now();
        publish_mode_marker(now);

        std_msgs::msg::Header hdr = msg->header;
        hdr.stamp = now;
        publish_cone_marker(hdr);
        publish_ttc_marker(hdr, min_ttc);
    }

    // -------------------------------------------------------------------------
    // Drive callbacks: simply gate on reactive_mode_active_
    // -------------------------------------------------------------------------
    void drive_pp_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        if (!reactive_mode_active_)
            drive_pub_->publish(*msg);
    }

    void drive_re_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        if (reactive_mode_active_)
            drive_pub_->publish(*msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveCombiner>());
    rclcpp::shutdown();
    return 0;
}