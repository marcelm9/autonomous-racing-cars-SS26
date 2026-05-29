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
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "msgs/msg/raceline.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
    this->declare_parameter<double>("min_lookahead", 0.5);
    this->declare_parameter<double>("max_lookahead", 2.0);
    this->declare_parameter<double>("wheelbase", 0.33);
    this->declare_parameter<double>("max_steering_angle", 0.42);
    this->declare_parameter<double>("min_speed", 1.5);
    this->declare_parameter<double>("max_speed", 4.0);
    this->declare_parameter<bool>("do_adaptive_speed_control", false);
    this->declare_parameter<double>("adaptive_control_slowing_factor", 1.0);
    this->declare_parameter<double>("adaptive_control_speed_up_factor", 1.0);

    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    // Cache the control parameters to avoid map lookups in the control loop
    min_lookahead_ = this->get_parameter("min_lookahead").as_double();
    max_lookahead_ = this->get_parameter("max_lookahead").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    min_speed_ = this->get_parameter("min_speed").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();
    do_adaptive_speed_control_ = this->get_parameter("do_adaptive_speed_control").as_bool();

    // Setup the parameter update callback for dynamic rqt/CLI tuning
    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters)
        {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto &param : parameters)
          {
            if (param.get_name() == "min_lookahead" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              min_lookahead_ = param.as_double();
            }
            else if (param.get_name() == "max_lookahead" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              max_lookahead_ = param.as_double();
            }
            else if (param.get_name() == "wheelbase" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              wheelbase_ = param.as_double();
            }
            else if (param.get_name() == "max_steering_angle" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              max_steering_angle_ = param.as_double();
            }
            else if (param.get_name() == "min_speed" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              min_speed_ = param.as_double();
            }
            else if (param.get_name() == "max_speed" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              max_speed_ = param.as_double();
            }
            else if (param.get_name() == "do_adaptive_speed_control" && param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            {
              do_adaptive_speed_control_ = param.as_bool();
            }
            else if (param.get_name() == "adaptive_control_slowing_factor" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              slow_down_factor = param.as_double();
            }
            else if (param.get_name() == "adaptive_control_speed_up_factor" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
              speed_up_factor = param.as_double();
            }
          }
          return result;
        });

    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        this->get_parameter("drive_topic").as_string(), 10);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/raceline_pp/goal_point", 10);
    nearest_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/raceline_pp/nearest_point", 10);
    car_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/raceline_pp/car_position", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/raceline/live_velocity_markers", 10);


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
    if (!raceline_error_initialized) {      
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

      std::cout << "init error vector" << std::endl;
      std::vector<double> err_hist(raceline_.size(), 0.0); 
      raceline_error_history = err_hist;
      raceline_error_initialized = true;

      this->distance_between_two_raceline_points = raceline_[1].s_m - raceline_[0].s_m;
    }

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Received raceline with %zu points, closed length %.2f m",
        raceline_.size(),
        pure_pursuit::computeClosedTrackLength(raceline_));
  }


  void re_init_raceline_max_points()
  {
    if (!raceline_.empty())
    {
      // initialize to the first point's velocity
      min_mps_ = raceline_.front().vx_mps;
      max_mps_ = raceline_.front().vx_mps;

      // find the min and max velocities in the newly received raceline
      for (const auto &point : raceline_)
      {
        if (point.vx_mps < min_mps_)
        {
          min_mps_ = point.vx_mps;
        }
        if (point.vx_mps > max_mps_)
        {
          max_mps_ = point.vx_mps;
        }
      }

        
      RCLCPP_INFO(
          this->get_logger(),
          "Raceline speed limits (MPS, not real values! For real set 'min_speed' and 'max_speed'): Min = %.2f m/s, Max = %.2f m/s",
          min_mps_, max_mps_);
    }
  }


  // CONTROLLER CALLBACK
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

    const double lookahead_m = pure_pursuit::computeMpsDependentLookahead(
        min_mps_,
        max_mps_,
        raceline_[nearest_index].vx_mps,
        min_lookahead_,
        max_lookahead_);

    std::size_t goal_index =
        pure_pursuit::findLookaheadPointIndex(raceline_, nearest_index, lookahead_m);



    // automatic adaptive tune of speed

    // handle last error -> log
    
    // calc error
    double err = std::hypot(raceline_[nearest_index].x_m - car_x, raceline_[nearest_index].y_m - car_y);
    
    // set error
    std::size_t size = this->raceline_error_history.size();
    double factor = 1.0;
    std::size_t i = 0;
    if (err < 0.1) {
      factor = speed_up_factor;
      i = last_nearest_index;
    } else if (err > 0.2) {
      factor = this->slow_down_factor;
      double distance = (1.0 / distance_between_two_raceline_points);
      std::cout << "distance: " << distance <<  std::endl;
      int temp = static_cast<int>( std::round(static_cast<double>(last_nearest_index) - distance ));
      i = ( (temp % static_cast<int>(size)) + size ) % size;
    } else {
      factor = 1.0;
      i = last_nearest_index;
    }

    while (i != nearest_index) {
      this->raceline_error_history[i] = err;

      // adapt speed using the error profile
      if (do_adaptive_speed_control_) {
        // check error value
        //std::cout << "error: " << err << std::endl;
        
        this->raceline_[i].vx_mps = this->raceline_[i].vx_mps * factor;
      }
      i = (i + 1) % size;  // wrap around at end
    }



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
        wheelbase_,
        max_steering_angle_);

    double target_speed = min_speed_;
    if (!do_adaptive_speed_control_) {
      target_speed = pure_pursuit::computeMpsDependentSpeed(
          min_mps_,
          max_mps_,
          raceline_[nearest_index].vx_mps,
          min_speed_,
          max_speed_);
    } else {
      target_speed = raceline_[nearest_index].vx_mps;
    }

    std::cout << "raceline_speed: " << raceline_[nearest_index].vx_mps << " publishing this speed: " << target_speed << std::endl;
    this->publishMarkers(goal_map, nearest_index, car_x, car_y);
    this->publishDrive(steering_angle, target_speed);
    this->publishRacelineMarkers();
    this->re_init_raceline_max_points();
    this->last_nearest_index = nearest_index;

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

  void publishRacelineMarkers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    int skip_n = 30;
    for (std::size_t idx = 0; idx < raceline_.size(); ++idx)
    {
        // skip points: only process every nth point
        if (idx % skip_n != 0)
            continue;

        const auto &point = raceline_[idx];

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "raceline_velocity";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = point.x_m;
        marker.pose.position.y = point.y_m;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.18;

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << "v:" <<point.vx_mps << " e:" << raceline_error_history[idx];
        marker.text = ss.str();

        marker_array.markers.push_back(marker);
    }

      // Publish the MarkerArray
      marker_pub_->publish(marker_array);
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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Callback handle to keep the parameter event listener alive
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::vector<pure_pursuit::RacelinePoint> raceline_;
  std::string map_frame_;
  std::string base_frame_;
  double current_speed_mps_ = 0.0;
  bool raceline_error_initialized = false;
  std::vector<double> raceline_error_history;
  
  double min_mps_ = -1.0;
  double max_mps_ = -1.0;

  // Cached parameters
  double min_lookahead_;
  double max_lookahead_;
  double wheelbase_;
  double max_steering_angle_;
  double min_speed_;
  double max_speed_;
  bool do_adaptive_speed_control_ = false;
  int last_nearest_index = 0;
  double distance_between_two_raceline_points = 0;
  double speed_up_factor = 1.0;
  double slow_down_factor = 1.0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacelinePurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}