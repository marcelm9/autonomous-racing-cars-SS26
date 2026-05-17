#include "pure_pursuit/raceline_pure_pursuit.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace
{

constexpr const char * DEFAULT_RACELINE_PATH =
  "opt_raceline/opt_raceline_algo/outputs/crazy_map/traj_race_cl.csv";

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw_rad)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(yaw_rad / 2.0);
  quaternion.w = std::cos(yaw_rad / 2.0);
  return quaternion;
}

std::string resolveRacelinePath(const std::string & configured_path)
{
  namespace fs = std::filesystem;

  const fs::path path(configured_path);
  if (path.is_absolute() && fs::exists(path)) {
    return path.string();
  }

  const fs::path cwd_relative = fs::current_path() / path;
  if (fs::exists(cwd_relative)) {
    return cwd_relative.string();
  }

  const fs::path arc_workspace =
    fs::path("/arc2026/ws/src/your_code/arc_riders") / path;
  if (fs::exists(arc_workspace)) {
    return arc_workspace.string();
  }

  try {
    const fs::path opt_share = ament_index_cpp::get_package_share_directory("opt_raceline");
    const fs::path share_relative = opt_share.parent_path().parent_path() / path;
    if (fs::exists(share_relative)) {
      return share_relative.string();
    }
  } catch (const std::exception &) {
  }

  return configured_path;
}

}  // namespace

class RacelinePurePursuitNode : public rclcpp::Node
{
public:
  RacelinePurePursuitNode()
  : Node("raceline_pure_pursuit_node"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    this->declare_parameter<std::string>("raceline_path", DEFAULT_RACELINE_PATH);
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
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/raceline/path", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->get_parameter("odom_topic").as_string(),
      10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto & velocity = msg->twist.twist.linear;
        current_speed_mps_ = std::hypot(velocity.x, velocity.y);
      });

    loadRaceline();

    const double control_rate_hz = std::max(1.0, this->get_parameter("control_rate_hz").as_double());
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / control_rate_hz)),
      std::bind(&RacelinePurePursuitNode::controlTimerCallback, this));
  }

private:
  void loadRaceline()
  {
    const auto configured_path = this->get_parameter("raceline_path").as_string();
    raceline_path_ = resolveRacelinePath(configured_path);

    try {
      raceline_ = pure_pursuit::parseRacelineCsv(raceline_path_);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load raceline CSV '%s': %s", raceline_path_.c_str(), ex.what());
      raceline_.clear();
      return;
    }

    raceline_path_msg_ = makePathMessage();
    RCLCPP_INFO(
      this->get_logger(),
      "Loaded raceline CSV '%s' with %zu points, closed length %.2f m",
      raceline_path_.c_str(),
      raceline_.size(),
      pure_pursuit::computeClosedTrackLength(raceline_));
  }

  nav_msgs::msg::Path makePathMessage() const
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = map_frame_;

    for (const auto & point : raceline_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = map_frame_;
      pose.pose.position.x = point.x_m;
      pose.pose.position.y = point.y_m;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = yawToQuaternion(point.psi_rad);
      path_msg.poses.push_back(pose);
    }

    return path_msg;
  }

  void controlTimerCallback()
  {
    if (raceline_.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No raceline loaded; not publishing drive commands");
      return;
    }

    geometry_msgs::msg::TransformStamped map_to_base;
    try {
      map_to_base = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Could not transform %s to %s: %s", map_frame_.c_str(), base_frame_.c_str(), ex.what());
      publishRacelinePath();
      return;
    }

    const double car_x = map_to_base.transform.translation.x;
    const double car_y = map_to_base.transform.translation.y;

    const std::size_t nearest_index =
      pure_pursuit::findNearestPointIndex(raceline_, car_x, car_y);
    const double lookahead_m = pure_pursuit::computeSpeedDependentLookahead(
      current_speed_mps_,
      this->get_parameter("min_lookahead").as_double(),
      this->get_parameter("max_lookahead").as_double(),
      this->get_parameter("lookahead_gain").as_double());
    const std::size_t goal_index =
      pure_pursuit::findLookaheadPointIndex(raceline_, nearest_index, lookahead_m);

    geometry_msgs::msg::PointStamped goal_map;
    goal_map.header.frame_id = map_frame_;
    goal_map.header.stamp = this->now();
    goal_map.point.x = raceline_[goal_index].x_m;
    goal_map.point.y = raceline_[goal_index].y_m;
    goal_map.point.z = 0.0;

    geometry_msgs::msg::PointStamped goal_base;
    try {
      tf_buffer_->transform(goal_map, goal_base, base_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Could not transform raceline goal point to %s: %s", base_frame_.c_str(), ex.what());
      publishRacelinePath();
      return;
    }

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
    publishRacelinePath();
  }

  void publishMarkers(
    const geometry_msgs::msg::PointStamped & goal_map,
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
  }

  void publishRacelinePath()
  {
    raceline_path_msg_.header.stamp = this->now();
    for (auto & pose : raceline_path_msg_.poses) {
      pose.header.stamp = raceline_path_msg_.header.stamp;
    }
    path_pub_->publish(raceline_path_msg_);
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr nearest_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr car_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::vector<pure_pursuit::RacelinePoint> raceline_;
  nav_msgs::msg::Path raceline_path_msg_;
  std::string raceline_path_;
  std::string map_frame_;
  std::string base_frame_;
  double current_speed_mps_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacelinePurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
