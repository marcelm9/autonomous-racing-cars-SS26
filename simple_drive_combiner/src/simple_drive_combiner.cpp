#include <string>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class StrategyDriveCombiner : public rclcpp::Node
{
public:
    StrategyDriveCombiner()
        : Node("strategy_drive_combiner"),
          reactive_mode_active_(false) // Default fallback mode
    {
        // Declare string tokens used by your strategist file
        this->declare_parameter<std::string>("pure_pursuit_strategy_token", "pure_pursuit");
        this->declare_parameter<std::string>("reactive_strategy_token", "reactive");

        // Topic parameters
        this->declare_parameter<std::string>("strategy_topic", "/strategist/strategy");
        this->declare_parameter<std::string>("pp_input_topic", "/drive_pp");
        this->declare_parameter<std::string>("re_input_topic", "/drive_re");
        this->declare_parameter<std::string>("drive_output_topic", "/drive");

        // Fetch string tokens
        pp_token_ = this->get_parameter("pure_pursuit_strategy_token").as_string();
        re_token_ = this->get_parameter("reactive_strategy_token").as_string();

        // Subscribers
        strategy_sub_ = this->create_subscription<std_msgs::msg::String>(
            this->get_parameter("strategy_topic").as_string(), 10,
            std::bind(&StrategyDriveCombiner::strategy_callback, this, _1));

        drive_pp_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            this->get_parameter("pp_input_topic").as_string(), 10,
            std::bind(&StrategyDriveCombiner::drive_pp_callback, this, _1));

        drive_re_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            this->get_parameter("re_input_topic").as_string(), 10,
            std::bind(&StrategyDriveCombiner::drive_re_callback, this, _1));

        // Publisher
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            this->get_parameter("drive_output_topic").as_string(), 10);

        RCLCPP_INFO(this->get_logger(), "Strategy Drive Combiner running. Listening for tokens: '%s' and '%s'", 
                    pp_token_.c_str(), re_token_.c_str());
    }

private:
    // State
    bool reactive_mode_active_;

    // Configurable tokens matching your strategy file lines
    std::string pp_token_;
    std::string re_token_;

    // ROS Handles
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr strategy_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pp_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_re_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    // -------------------------------------------------------------------------
    // Strategy Callback - Directly flips the steering gate based on text names
    // -------------------------------------------------------------------------
    void strategy_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string &incoming_strategy = msg->data;

        if (incoming_strategy == pp_token_ && reactive_mode_active_)
        {
            reactive_mode_active_ = false;
            RCLCPP_INFO(this->get_logger(), "Strategy Event -> Flipped multiplexer to: PURE PURSUIT");
        }
        else if (incoming_strategy == re_token_ && !reactive_mode_active_)
        {
            reactive_mode_active_ = true;
            RCLCPP_INFO(this->get_logger(), "Strategy Event -> Flipped multiplexer to: REACTIVE MODE");
        }

        // Keep a throttled terminal diagnostic running
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Current Active Strategy Stream: %s | Routing Active Node: %s",
                             incoming_strategy.c_str(),
                             reactive_mode_active_ ? "REACTIVE" : "PURE_PURSUIT");
    }

    // -------------------------------------------------------------------------
    // Multiplexing Drive Pipelines
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