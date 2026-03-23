#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <limits>

#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    // Constructor
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

        ebrake_engaged_pub = this->create_publisher<std_msgs::msg::Bool>("/ebrake_engaged_pub", 10);

        ebrake_vis_pub = this->create_publisher<visualization_msgs::msg::Marker>("/ebrake_marker", 10);
        
        /* ---- services ---- */
        reset_service = this->create_service<std_srvs::srv::Trigger>( "/reset_safety_node", std::bind(&Safety::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    
        /* ---- declared parameters ----*/
        this->declare_parameter<float>("ttc_threshold", 0.7);

        // publish green marker once
        publish_ebrake_marker();
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

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ebrake_engaged_pub;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ebrake_vis_pub;

    // variables

    sensor_msgs::msg::LaserScan scan_msg;

    float min_TTC = std::numeric_limits<float>::infinity();
    bool e_brake = false;
    double measured_odometry_speed = 0.0;
    double requested_drive_speed = 0.0;

    ackermann_msgs::msg::AckermannDriveStamped drive_msg = ackermann_msgs::msg::AckermannDriveStamped();


    /* ---- CALLBACKS ---- */

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        this->measured_odometry_speed = msg->twist.twist.linear.x;
    }


    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        publish_ebrake_marker();
        this->scan_msg = *scan_msg;

        auto [min_ttc, min_distance] = calc_min_ttc(false); 

        if (min_ttc < this->get_parameter("ttc_threshold").as_double()) {
            this->e_brake = true;
            std::cout << "e-brake engaged" << std::endl;
        }
        this->min_TTC = min_ttc;

        // Todo put basic info prompts into one topic
        std_msgs::msg::Float64 ttc_msg;
        ttc_msg.data = this->min_TTC;
        ttc_pub->publish(ttc_msg);

        std_msgs::msg::Float64 min_distance_msg;
        min_distance_msg.data = min_distance;
        min_dist_obs_pub->publish(min_distance_msg);

        std_msgs::msg::Bool safety_brake;
        safety_brake.data = this->e_brake;
        ebrake_engaged_pub->publish(safety_brake);
        
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
        (void)request;
        // recheck TTC with desired velocity
        auto [min_TTC, _] = calc_min_ttc(true);

        if (min_TTC < this->get_parameter("ttc_threshold").as_double()) { 
            response->success = false;
            response->message = "E-Brake cannot be disenganged. TTC is still too low!";
        } else {
            this->e_brake = false;
            response->success = true;
            response->message = "E-Brake has successfully been disengaged";
            publish_ebrake_marker();
        }
    }


    /* ---- FUNCTIONS ---- */

    std::pair<float, float> calc_min_ttc(bool use_desired_speed)
    {
        double angle_increment = this->scan_msg.angle_increment;
        double angle_min = this->scan_msg.angle_min;
        std::vector<float> ranges = this->scan_msg.ranges;

        double speed = 0.0;
        if (use_desired_speed) {
            speed = this->requested_drive_speed;
        } else {
            speed = this->measured_odometry_speed;
        }

         // iterate through all lasers and project
        int num_lasers = ranges.size();
        double min_TTC = std::numeric_limits<float>::infinity();
        double min_distance = std::numeric_limits<float>::infinity();


        for (int i = 0; i < num_lasers; i++)  {
            double angle = i * angle_increment + angle_min;
            // Consider only window in front of car
            if (angle < -0.087 || angle > 0.087) { 
                continue; 
            }
            double projection = std::max(0.0, speed * std::cos(angle));
            double ttc;
            if (projection == 0.0) {
                ttc = std::numeric_limits<float>::infinity();
            } else {
                ttc = ranges[i] / projection;
            }

            if (ttc < min_TTC) {
                min_TTC = ttc;
            }

            if (ranges[i] < min_distance) {
                min_distance = ranges[i];
            }

        }
        return {min_TTC, min_distance};
    }

    // see marker doc: https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html
    void publish_ebrake_marker()
    {
        // create message
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "ego_racecar/base_link";
        marker.header.stamp = rclcpp::Clock().now();

        marker.ns = "basic_shapes";
        marker.id = 0;

        marker.type = visualization_msgs::msg::Marker::SPHERE;

        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = 0.1;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0.2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        if (this->e_brake) {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
        } else {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
        }

        ebrake_vis_pub->publish(marker);
    }


};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}