#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <limits>

#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /* ---- subscribers ---- */
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&Safety::odom_callback, this, std::placeholders::_1));

        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1)); 

        requested_drive_speed_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/drive_in", 10, std::bind(&Safety::requested_drive_speed_callback, this, std::placeholders::_1)); 
           
        
        /* ---- publishers ---- */
        drive_speed_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_out", 10);
        
        ttc_pub = this->create_publisher<std_msgs::msg::Float64>("/ttc_pub", 10);

        min_dist_obs_pub = this->create_publisher<std_msgs::msg::Float64>("/min_dist_obs_pub", 10);

        ttc_engaged_pub = this->create_publisher<std_msgs::msg::Bool>("/ttc_engaged_pub", 10);
        
        /* ---- services ---- */
        reset_service = this->create_service<std_srvs::srv::Trigger>( "reset_safety", std::bind(&Safety::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:

    // declarations:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr requested_drive_speed_sub;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_speed_pub;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ttc_pub;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr min_dist_obs_pub;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ttc_engaged_pub;

    // variables
    float threshold = 0.5;

    float min_TTC = std::numeric_limits<float>::infinity();
    bool e_brake = false;
    double measured_odometry_speed = 0.0;
    float requested_drive_speed = 0.0;

    ackermann_msgs::msg::AckermannDriveStamped drive_msg = ackermann_msgs::msg::AckermannDriveStamped();


    /* ---- CALLBACKS ---- */

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        this->measured_odometry_speed = msg->twist.twist.linear.x;
    }


    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        // angle increment:
        float angle_increment = scan_msg->angle_increment;
        float angle_min = scan_msg->angle_min;
        // ranges:
        std::vector<float> ranges = scan_msg->ranges;

        // iterate through all lasers and project
        int num_lasers = ranges.size();
        float angle_of_min_TTC = 0;
        
        for (int i = 0; i < num_lasers; i++)  {
            float angle = i * angle_increment + angle_min;
            float projection = std::max(0.0, this->measured_odometry_speed * std::cos(angle));
            float TTC = ranges[i] / projection;
            
            if (TTC < this->min_TTC) {
                this->min_TTC = TTC;
                angle_of_min_TTC = angle;
            }

            if (TTC < threshold) {
                this->e_brake = true;
                std::cout << "e-brake engaged" << std::endl;
                break;
            }
        }
        std::cout << "Min TTC: " << this->min_TTC << " with ANGLE: " << angle_of_min_TTC << std::endl;

        std_msgs::msg::Float64 ttc_msg;
        ttc_msg.data = this->min_TTC;
        ttc_pub->publish(ttc_msg);


        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        if (this->e_brake) {  // call publisher with ZERO velocity
            drive_msg.drive.speed = 0.0;
            drive_msg.drive.steering_angle = 0.0;
        } else {  // pass on desired velocity to publisher
            drive_msg = this->drive_msg;
        }

        drive_speed_pub->publish(drive_msg);
    }


    void requested_drive_speed_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg)
    {
        this->requested_drive_speed = msg->drive.speed;
        this->drive_msg = *msg;
    }


    void reset_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (this->min_TTC < this->threshold) {  // recheck if driving is save now
            this->e_brake = false;
            response->success = false;
            response->message = "E-Brake cannot be disenganged. TTC is still too low!";
        } else {
            response->success = true;
            response->message = "E-Brake has successfully been disengaged";
        }
    }


};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}