
#include <cstdio>
#include "./../include/pure_pursuit/controller.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

controller::controller() : Node("controller")
{
  /* ---- subscribers ---- */
  path_sub = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&controller::path_callback, this, std::placeholders::_1));

  /* ---- publishers ---- */
  drive_speed_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

  std::cout << "in constructor" << std::endl;  

  // publish diagnostics once per second
  
  // variables

  // timers
  pure_persuit_timer = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&controller::pure_persuit_timer_callback, this)
  );

  // TF buffer initilization
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}


void controller::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  //RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", msg->poses.size());

  this->current_path = *msg;
  this->path_received_ = true;
}

void controller::pure_persuit_timer_callback()
{
  std::cout << "in timer callback" << std::endl;
  if (!path_received_) return;

  geometry_msgs::msg::TransformStamped map_to_base_link;

  try {
      // 1. Get the current position of the car in the map
      // Target: map, Source: base_link
      map_to_base_link = tf_buffer_->lookupTransform(
          "map", "ego_racecar/base_link", tf2::TimePointZero);
          
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
      return;
  }

  double car_x = map_to_base_link.transform.translation.x;
  double car_y = map_to_base_link.transform.translation.y;

  float min_dist = std::numeric_limits<float>::infinity();
  int closest_index = -1;
  for (size_t i = 0; i < this->current_path.poses.size(); ++i)
  {
      double path_x = this->current_path.poses[i].pose.position.x;
      double path_y = this->current_path.poses[i].pose.position.y;
      double dist = std::sqrt(std::pow(path_x - car_x, 2) + std::pow(path_y - car_y, 2));
      if (dist < min_dist)
      {
          min_dist = dist;
          closest_index = i;
      }
  }
  if (closest_index != -1)
  {
      std::cout << "Closest path point: (" << this->current_path.poses[closest_index].pose.position.x
                << ", " << this->current_path.poses[closest_index].pose.position.y << ")" << std::endl;
  }

  float min_diff = std::numeric_limits<float>::infinity();
  std::vector<geometry_msgs::msg::PoseStamped> possible_goal_points = std::vector<geometry_msgs::msg::PoseStamped>(this->current_path.poses.begin() + closest_index, this->current_path.poses.end());
  geometry_msgs::msg::PoseStamped goal_point;

  for (const auto & path_point : possible_goal_points)
  {
      double path_x = path_point.pose.position.x;
      double path_y = path_point.pose.position.y;
      double dist = std::sqrt(std::pow(path_x - car_x, 2) + std::pow(path_y - car_y, 2));
      if (std::abs(dist - L) < min_diff)
      {
          min_diff = std::abs(dist - L);
          goal_point = path_point;
      }
  }
  
  geometry_msgs::msg::PointStamped goal_map_frame;
  goal_map_frame.header.frame_id = "map";
  goal_map_frame.point = goal_point.pose.position;

    geometry_msgs::msg::PointStamped goal_car_frame;

    try {
        
        tf_buffer_->transform(goal_map_frame, goal_car_frame, "ego_racecar/base_link");
        
        
        double lx = goal_car_frame.point.x; // Distance forward from car
        double ly = goal_car_frame.point.y; // Distance left/right from car center
        
       
        
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Goal transform failed: %s", ex.what());
    }
  
    // Calc steering angle using pure pursuit geometry using lx and ly
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller>());
  rclcpp::shutdown();
  return 0;
}
