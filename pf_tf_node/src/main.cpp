//
// Created by felix on 4/12/23.
//
#if !defined(__has_include)
#   define __has_include(x) 0
#endif
#include <memory>
#include "rclcpp/rclcpp.hpp"
// Some versions are lacking .h/.hpp header files,
// see https://github.com/ros2/geometry2/pull/720
// TL;DR: Newer versions will contain only .hpp.
#if __has_include("tf2_ros/transform_broadcaster.hpp")
#   include "tf2_ros/transform_broadcaster.hpp"
#else
#   include "tf2_ros/transform_broadcaster.h"
#endif
#if __has_include("tf2_ros/transform_listener.hpp")
#   include "tf2_ros/transform_listener.hpp"
#else
#   include "tf2_ros/transform_listener.h"
#endif
#if __has_include("tf2_ros/buffer.hpp")
#   include "tf2_ros/buffer.hpp"
#else
#   include "tf2_ros/buffer.h"
#endif
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#   include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#   include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>

using namespace std::chrono_literals;

using std::placeholders::_1;

class DynamicTransform : public rclcpp::Node {
public:
    DynamicTransform(): Node("dynamic_transform") {
        tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ =
                std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 1, std::bind(&DynamicTransform::pose_callback, this, _1));

        auto update_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        update_param_desc.description = "Enable the update of the transformation tree";
        this->declare_parameter("update_tf_tree", true, update_param_desc);

        auto odom_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        odom_param_desc.description = "Frame that the odometry message points to";
        this->declare_parameter("odom_msg_frame", "laser", odom_param_desc);

        auto map_param_desc = rcl_interfaces::msg::ParameterDescriptor {};
        map_param_desc.description = "Map frame name";
        this->declare_parameter("map_frame", "map", map_param_desc);

        auto child_param_desc = rcl_interfaces::msg::ParameterDescriptor {};
        child_param_desc.description = "Child frame this node should publish a transform for";
        this->declare_parameter("child_frame", "odom", child_param_desc);
        
        timer_ = this->create_wall_timer( 12.5ms, std::bind(&DynamicTransform::timer_callback, this));
        has_transform_ = false;
    }

private:

    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Incoming odometry update");

        if (this->get_parameter("update_tf_tree").as_bool()) {
            try {
                tf2::Transform map_laser;
                tf2::fromMsg(msg->pose.pose, map_laser);

                tf2::Stamped<tf2::Transform> laser_map_tf(
                        map_laser.inverse(),
                        tf2::TimePoint(),
                        "laser"
                );

                geometry_msgs::msg::TransformStamped laser_map_ros = tf2::toMsg(laser_map_tf);

                auto map_odom_pose = tf_buffer_->transform(laser_map_ros,
                                                           this->get_parameter("child_frame").as_string());

                tf2::Stamped<tf2::Transform> map_odom;
                tf2::fromMsg(map_odom_pose, map_odom);
                tf2::Stamped<tf2::Transform> transform_stamped(
                        map_odom.inverse(),
                        tf2::getTimestamp(map_odom),
                        tf2::getFrameId(map_odom)
                );

                geometry_msgs::msg::TransformStamped message = tf2::toMsg(transform_stamped);
                message.child_frame_id = this->get_parameter("child_frame").as_string();
                message.header.frame_id = this->get_parameter("map_frame").as_string();

                current_message_ = message;
                has_transform_ = true;
            } catch (tf2::TransformException &e) {
                RCLCPP_WARN_STREAM(this->get_logger(), "transformation error: " << e.what());
            }
        }
    }
    
    void timer_callback() {
        if (has_transform_) {
            tf_broadcaster_->sendTransform(current_message_);
        }
    }

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    geometry_msgs::msg::TransformStamped current_message_;
    bool has_transform_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicTransform>());
    rclcpp::shutdown();
    return 0;
}
