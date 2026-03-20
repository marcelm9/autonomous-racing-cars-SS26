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
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1)); 

        drive_speed_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/drive_in", 10, std::bind(&Safety::speed_callback, this, std::placeholders::_1));

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, std::placeholders::_1)); 
        desired_speed_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&Safety::desired_speed_callback, this, std::placeholders::_1));
        
        // publishers:
        drive_speed_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_in", 10);
        
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
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_speed_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_speed_sub;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_speed_pub;


    double speed = 0.0;
    /// TODO: create ROS subscribers and publishers

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        std::cout << "IN drive callback:\n";

        // Position
        std::cout << "  Position: x=" << msg->pose.pose.position.x
                << ", y=" << msg->pose.pose.position.y
                << ", z=" << msg->pose.pose.position.z << "\n";

        // Orientation (quaternion)
        std::cout << "  Orientation: x=" << msg->pose.pose.orientation.x
                << ", y=" << msg->pose.pose.orientation.y
                << ", z=" << msg->pose.pose.orientation.z
                << ", w=" << msg->pose.pose.orientation.w << "\n";

        // Linear velocity
        std::cout << "  Linear velocity: x=" << msg->twist.twist.linear.x
                << ", y=" << msg->twist.twist.linear.y
                << ", z=" << msg->twist.twist.linear.z << "\n";

        // Angular velocity
        std::cout << "  Angular velocity: x=" << msg->twist.twist.angular.x
                << ", y=" << msg->twist.twist.angular.y
                << ", z=" << msg->twist.twist.angular.z << "\n";


        
    }


    void desired_speed_callback(const geometry_msgs::msg::Twist::ConstSharedPtr desired_speed)
    {
        /*
        std::cout << "Received desired speed:\n";
        std::cout << "  linear x: " << desired_speed->linear.x << "\n";
        std::cout << "  linear y: " << desired_speed->linear.y << "\n";
        std::cout << "  linear z: " << desired_speed->linear.z << "\n";
        std::cout << "  angular x: " << desired_speed->angular.x << "\n";
        std::cout << "  angular y: " << desired_speed->angular.y << "\n";
        std::cout << "  angular z: " << desired_speed->angular.z << "\n";
        this->speed = 
        */
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC

        /// TODO: publish drive/brake message
    }

    void speed_callback(const ackermann_msgs::msg::AckermannDriveStamped speed_msg)
    {

    }


};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}