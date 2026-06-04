#include <string>
#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class StrategyDriveCombiner : public rclcpp::Node
{
public:
    StrategyDriveCombiner()
        : Node("strategy_drive_combiner"),
          reactive_mode_active_(false),
          distance_driven_in_reactive_(0.0),
          has_last_pos_(false),
          initial_pose_published_(false)
    {
        // Declare parameters (Reconfigurable on-the-fly)
        this->declare_parameter<double>("reactive_distance_threshold_m", 5.0);
        this->declare_parameter<std::string>("reactive_strategy_token", "reactive");
        this->declare_parameter<std::string>("pure_pursuit_strategy_token", "pure_pursuit");

        // Topic parameters
        this->declare_parameter<std::string>("strategy_topic", "/strategist/strategy");
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("pp_input_topic", "/drive_pp");
        this->declare_parameter<std::string>("re_input_topic", "/drive_re");
        this->declare_parameter<std::string>("drive_output_topic", "/drive");
        this->declare_parameter<std::string>("initial_pose_topic", "/initialpose");

        // Fetch initial parameter configurations
        reactive_dist_thresh_ = this->get_parameter("reactive_distance_threshold_m").as_double();
        re_token_ = this->get_parameter("reactive_strategy_token").as_string();
        pp_token_ = this->get_parameter("pure_pursuit_strategy_token").as_string();

        // Register Dynamic Reconfigure parameter callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&StrategyDriveCombiner::parameters_callback, this, std::placeholders::_1));

        // Subscribers
        strategy_sub_ = this->create_subscription<std_msgs::msg::String>(
            this->get_parameter("strategy_topic").as_string(), 10,
            std::bind(&StrategyDriveCombiner::strategy_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_topic").as_string(), 10,
            std::bind(&StrategyDriveCombiner::odom_callback, this, _1));

        drive_pp_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            this->get_parameter("pp_input_topic").as_string(), 10,
            std::bind(&StrategyDriveCombiner::drive_pp_callback, this, _1));

        drive_re_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            this->get_parameter("re_input_topic").as_string(), 10,
            std::bind(&StrategyDriveCombiner::drive_re_callback, this, _1));

        // Publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            this->get_parameter("drive_output_topic").as_string(), 10);

        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            this->get_parameter("initial_pose_topic").as_string(), 10);

        // Publish the initial pose setup sequence right away at startup
        // Using a short one-shot timer ensures the publisher is fully initialized within the ROS mesh network
        init_pose_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&StrategyDriveCombiner::publish_startup_pose, this));

        RCLCPP_INFO(this->get_logger(), "Initialized. Mode: PURE PURSUIT. Limit: %.2fm", reactive_dist_thresh_);
    }

private:
    // State Flags
    bool reactive_mode_active_;
    double distance_driven_in_reactive_;
    bool has_last_pos_;
    double last_x_, last_y_;
    bool initial_pose_published_;

    // Parameters
    double reactive_dist_thresh_;
    std::string re_token_;
    std::string pp_token_;

    // ROS Core Handles
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr strategy_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pp_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_re_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::TimerBase::SharedPtr init_pose_timer_;

    // -------------------------------------------------------------------------
    // Dynamic Parameter Reconfiguration Callback
    // -------------------------------------------------------------------------
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters)
        {
            if (param.get_name() == "reactive_distance_threshold_m" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                reactive_dist_thresh_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Dynamic Param Update -> reactive_distance_threshold_m: %.2f meters", reactive_dist_thresh_);
            }
        }
        return result;
    }

    // -------------------------------------------------------------------------
    // One-shot Startup Initial Pose Broadcast
    // -------------------------------------------------------------------------
    void publish_startup_pose()
    {
        if (initial_pose_published_)
        {
            init_pose_timer_->cancel();
            return;
        }

        auto init_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        init_pose_msg.header.stamp = this->get_clock()->now();
        init_pose_msg.header.frame_id = "map"; // Standard localization frame

        // Change these coordinates to wherever your map's origin starter line sits
        init_pose_msg.pose.pose.position.x = 0.0;
        init_pose_msg.pose.pose.position.y = 0.0;
        init_pose_msg.pose.pose.position.z = 0.0;
        
        // Face forward along the X axis (Quaternion identity)
        init_pose_msg.pose.pose.orientation.w = 1.0;

        // Populate a standard default diagonal identity covariance matrix
        for(int i = 0; i < 36; i += 7) {
            init_pose_msg.pose.covariance[i] = 0.25; 
        }

        initial_pose_pub_->publish(init_pose_msg);
        RCLCPP_INFO(this->get_logger(), "🎯 Sent Fixed Initial Pose Estimate to /initialpose");
        
        initial_pose_published_ = true;
        init_pose_timer_->cancel();
    }

    // -------------------------------------------------------------------------
    // Strategy Input Callback
    // -------------------------------------------------------------------------
    void strategy_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Only allow switching INTO reactive mode via the strategist topic.
        // Returning to pure pursuit is handled autonomously via tracking odometry distance.
        if (msg->data == re_token_ && !reactive_mode_active_)
        {
            reactive_mode_active_ = true;
            distance_driven_in_reactive_ = 0.0; // Reset trip odometer
            RCLCPP_INFO(this->get_logger(), "⚡ Strategy Event -> TRIGGERED REACTIVE MODE. Timeout in: %.2f meters.", reactive_dist_thresh_);
        }
        else if (msg->data == pp_token_ && reactive_mode_active_)
        {
            // Optional: If the strategist explicitly tells us to exit early, honor it
            reactive_mode_active_ = false;
            RCLCPP_INFO(this->get_logger(), "📋 Strategy Event -> Early exit to PURE PURSUIT via Strategist request.");
        }
    }

    // -------------------------------------------------------------------------
    // Odometry Tracker: Accumulates distance while inside reactive state
    // -------------------------------------------------------------------------
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;

        if (has_last_pos_ && reactive_mode_active_)
        {
            // Calculate delta distance moved during this timestep
            double delta_dist = std::hypot(current_x - last_x_, current_y - last_y_);
            distance_driven_in_reactive_ += delta_dist;

            // Check if the parameter target has been surpassed
            if (distance_driven_in_reactive_ >= reactive_dist_thresh_)
            {
                reactive_mode_active_ = false;
                RCLCPP_INFO(this->get_logger(), "⏱️ Odometer Limit Reached (%.2fm >= %.2fm)! Returning to PURE PURSUIT.", 
                            distance_driven_in_reactive_, reactive_dist_thresh_);
            }
        }

        last_x_ = current_x;
        last_y_ = current_y;
        has_last_pos_ = true;

        if (reactive_mode_active_)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "Reactive Odometer: %.2f / %.2f meters", 
                                 distance_driven_in_reactive_, reactive_dist_thresh_);
        }
    }

    // -------------------------------------------------------------------------
    // Multiplexer Logic Gating
    // -------------------------------------------------------------------------
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
    rclcpp::spin(std::make_shared<StrategyDriveCombiner>());
    rclcpp::shutdown();
    return 0;
}