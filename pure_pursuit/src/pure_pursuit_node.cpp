#include "pure_pursuit/pure_pursuit.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "msgs/msg/raceline.hpp"

namespace
{

  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw_rad)
  {
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = std::sin(yaw_rad / 2.0);
    quaternion.w = std::cos(yaw_rad / 2.0);
    return quaternion;
  }

} // namespace

class RacelinePurePursuitNode : public rclcpp::Node
{
public:
  RacelinePurePursuitNode()
      : Node("pure_pursuit"),
        tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
        tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "ego_racecar/base_link");
    this->declare_parameter<std::string>("drive_topic", "/drive");
    this->declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    this->declare_parameter<double>("control_rate_hz", 20.0);
    this->declare_parameter<double>("min_lookahead", 0.7);
    this->declare_parameter<double>("max_lookahead", 3.0);
    this->declare_parameter<double>("lookahead_gain", 0.35);
    this->declare_parameter<double>("wheelbase", 0.33);
    this->declare_parameter<double>("max_steering_angle", 0.42);
    this->declare_parameter<double>("target_speed_scaling", 0.65);
    this->declare_parameter<double>("min_target_speed", 1.0);
    this->declare_parameter<double>("max_target_speed", 6.0);
    this->declare_parameter<double>("curvature_speed_gain", 0.5);

    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        this->get_parameter("drive_topic").as_string(), 10);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/raceline_pp/goal_point", 10);
    nearest_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/raceline_pp/nearest_point", 10);
    car_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/raceline_pp/car_position", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("odom_topic").as_string(),
        10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
          const auto &velocity = msg->twist.twist.linear;
          current_speed_mps_ = std::hypot(velocity.x, velocity.y);
        });

    raceline_sub_ = this->create_subscription<msgs::msg::Raceline>(
        "/raceline/trajectory",
        10,
        std::bind(&RacelinePurePursuitNode::racelineCallback, this, std::placeholders::_1));

    const double control_rate_hz = std::max(1.0, this->get_parameter("control_rate_hz").as_double());
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / control_rate_hz)),
        std::bind(&RacelinePurePursuitNode::controlTimerCallback, this));

  }

private:
  void racelineCallback(const msgs::msg::Raceline::SharedPtr msg)
  {
    raceline_.clear();
    raceline_.reserve(msg->points.size());

    for (const auto &p : msg->points)
    {
      pure_pursuit::RacelinePoint point;
      point.s_m = p.s_m;
      point.x_m = p.x_m;
      point.y_m = p.y_m;
      point.psi_rad = p.psi_rad;
      point.kappa_radpm = p.kappa_radpm;
      point.vx_mps = p.vx_mps;
      point.ax_mps2 = p.ax_mps2;
      raceline_.push_back(point);
    }

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Received raceline with %zu points, closed length %.2f m",
        raceline_.size(),
        pure_pursuit::computeClosedTrackLength(raceline_));
  }

  void controlTimerCallback()
  {
    if (raceline_.empty())
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "No raceline received yet; not publishing drive commands");
      return;
    }

    geometry_msgs::msg::TransformStamped map_to_base;
    try
    {
      map_to_base = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Could not transform %s to %s: %s", map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    const double car_x = map_to_base.transform.translation.x;
    const double car_y = map_to_base.transform.translation.y;

    const std::size_t nearest_index =
        pure_pursuit::findNearestPointIndex(raceline_, car_x, car_y);
    const double lookahead_m = pure_pursuit::computeSpeedDependentLookahead(
        raceline_[nearest_index].vx_mps,
        this->get_parameter("min_lookahead").as_double(),
        this->get_parameter("max_lookahead").as_double(),
        this->get_parameter("lookahead_gain").as_double(),
        this->get_parameter("min_target_speed").as_double(),
        this->get_parameter("max_target_speed").as_double()
      );
    std::size_t goal_index =
        pure_pursuit::findLookaheadPointIndex(raceline_, nearest_index, lookahead_m);

    // std::size_t goal_index = computeCurvatureDependentLookahead(raceline_, nearest_index, this->get_parameter("min_lookahead").as_double(), this->get_parameter("max_lookahead").as_double(), this->get_parameter("lookahead_gain").as_double());

    geometry_msgs::msg::PointStamped goal_map;
    goal_map.header.frame_id = map_frame_;
    // Use latest available TF for the control transform. Stamping with now()
    // can be a few milliseconds ahead of the newest simulator TF sample.
    goal_map.header.stamp.sec = 0;
    goal_map.header.stamp.nanosec = 0;
    goal_map.point.x = raceline_[goal_index].x_m;
    goal_map.point.y = raceline_[goal_index].y_m;
    goal_map.point.z = 0.0;

    geometry_msgs::msg::PointStamped goal_base;
    try
    {
      tf_buffer_->transform(goal_map, goal_base, base_frame_);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Could not transform raceline goal point to %s: %s", base_frame_.c_str(), ex.what());
      return;
    }
    goal_map.header.stamp = this->now();

    const double steering_angle = pure_pursuit::computePurePursuitSteering(
        goal_base.point.x,
        goal_base.point.y,
        this->get_parameter("wheelbase").as_double(),
        this->get_parameter("max_steering_angle").as_double());

    double target_speed = raceline_[goal_index].vx_mps *
                          this->get_parameter("target_speed_scaling").as_double();
    target_speed = pure_pursuit::computeCurvatureLimitedSpeed(
        target_speed,
        raceline_[goal_index].kappa_radpm,
        this->get_parameter("curvature_speed_gain").as_double(),
        this->get_parameter("min_target_speed").as_double());
    target_speed = std::clamp(
        target_speed,
        this->get_parameter("min_target_speed").as_double(),
        this->get_parameter("max_target_speed").as_double());

    publishMarkers(goal_map, nearest_index, car_x, car_y);
    publishDrive(steering_angle, target_speed);

    // log Pos Error
    
  }

  void publishMarkers(
      const geometry_msgs::msg::PointStamped &goal_map,
      std::size_t nearest_index,
      double car_x,
      double car_y)
  {
    goal_pub_->publish(goal_map);

    geometry_msgs::msg::PointStamped nearest;
    nearest.header.frame_id = map_frame_;
    nearest.header.stamp = goal_map.header.stamp;
    nearest.point.x = raceline_[nearest_index].x_m;
    nearest.point.y = raceline_[nearest_index].y_m;
    nearest.point.z = 0.0;
    nearest_pub_->publish(nearest);

    geometry_msgs::msg::PointStamped car;
    car.header.frame_id = map_frame_;
    car.header.stamp = goal_map.header.stamp;
    car.point.x = car_x;
    car.point.y = car_y;
    car.point.z = 0.0;
    car_pub_->publish(car);
  }

  void publishDrive(double steering_angle, double target_speed)
  {
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.header.frame_id = base_frame_;
    drive_msg.drive.steering_angle = static_cast<float>(steering_angle);
    drive_msg.drive.speed = static_cast<float>(target_speed);
    drive_pub_->publish(drive_msg);
    // std::cout << "steering angle: " << steering_angle << std::endl;
    // std::cout << "target speed: " << target_speed << std::endl;
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<msgs::msg::Raceline>::SharedPtr raceline_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr nearest_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr car_pub_;

  std::vector<pure_pursuit::RacelinePoint> raceline_;
  std::string map_frame_;
  std::string base_frame_;
  double current_speed_mps_ = 0.0;
  bool raceline_error_initialized = false;
  std::vector<double> raceline_error_history;
  
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacelinePurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}