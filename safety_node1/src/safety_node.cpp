#include <algorithm>
#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        // subscribers:

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&Safety::odom_callback, this, std::placeholders::_1));

        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1)); 

        //drive_speed_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/drive_in", 10, std::bind(&Safety::speed_callback, this, std::placeholders::_1)); 
            
        // publishers:
        drive_speed_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers
        
    }

private:

    // declarations:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_speed_pub;


    double speed = 0.0;
    float threshold = 1.0;
    /// TODO: create ROS subscribers and publishers

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        this->speed = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC

        /// TODO: publish drive/brake message

        // angle increment:
        float angle_increment = scan_msg->angle_increment;
        float angle_min = scan_msg->angle_min;
        // ranges:
        std::vector<float> ranges = scan_msg->ranges;
        int num_lasers = ranges.size();
        
        // iterate through all lasers and project
        bool e_brake = false;
        for (int i = 0; i < num_lasers; i++)  {
            float angle = i * angle_increment + angle_min;
            float x_comp = (-1.0) * std::cos(angle) * ranges[i];
            float projection = std::max(0.0, this->speed * x_comp);
            float TTC = ranges[i] / projection;
            if (TTC < threshold) {
                e_brake = true;
                break;
            }
        }

        geometry_msgs::msg::Twist msg;
        if (e_brake) {  // call publisher with ZERO velocity

            msg.linear.x = 0.0;

        } else {  // pass on desired velocity to publisher

            msg.linear.x = this->speed;

        }

        drive_speed_pub->publish(msg);
    }


};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}