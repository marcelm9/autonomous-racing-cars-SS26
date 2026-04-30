#include "./../include/pure_pursuit/controller.hpp"

controller::controller() : Node("controller")
{
  /* ---- subscribers ---- */
  path_sub = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&controller::path_callback, this, std::placeholders::_1));

  /* ---- publishers ---- */
  drive_speed_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
  diagnostics_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  std::cout << "in constructor" << std::endl;  

  // publish
  goal_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/goal_point", 10);
  
  // variables
  this->declare_parameter<double>("LOOK_AHEAD_DISTANCE", 2.2);
  
  // timers
  pure_persuit_timer = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&controller::pure_persuit_timer_callback, this)
  );

  /* diagnostics_timer = this->create_wall_timer(
    std::chrono::milliseconds(1000), 
    std::bind(&controller::diagnostics_timer_callback, this)
  ); */

  // TF buffer initilization
  this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}


void controller::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  //RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", msg->poses.size());

  this->current_path = *msg;
  this->path_received_ = true;
}

void controller::pure_persuit_timer_callback()
{
  if (!this->path_received_) {
    std::cout << "NO PATH RECEIVED, RETURNING" << std::endl;
    return;
  }
  geometry_msgs::msg::TransformStamped map_to_base_link;

  try {
      // 1. Get the current position of the car in the map
      // Target: map, Source: base_link
      map_to_base_link = this->tf_buffer_->lookupTransform(
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
      //std::cout << "Closest path point: (" << this->current_path.poses[closest_index].pose.position.x << ", " << this->current_path.poses[closest_index].pose.position.y << ")" << std::endl;
  }

  float min_diff = std::numeric_limits<float>::infinity();
  std::vector<geometry_msgs::msg::PoseStamped> possible_goal_points = std::vector<geometry_msgs::msg::PoseStamped>(this->current_path.poses.begin() + closest_index, this->current_path.poses.end());
  geometry_msgs::msg::PoseStamped goal_point;

  for (const auto & path_point : possible_goal_points)
  {
      double path_x = path_point.pose.position.x;
      double path_y = path_point.pose.position.y;
      double dist = std::sqrt(std::pow(path_x - car_x, 2) + std::pow(path_y - car_y, 2));
      if (std::abs(dist - this->get_parameter("LOOK_AHEAD_DISTANCE").as_double()) < min_diff)
      {
          min_diff = std::abs(dist - this->get_parameter("LOOK_AHEAD_DISTANCE").as_double());
          goal_point = path_point;
          auto goal_msg = geometry_msgs::msg::PointStamped();

          goal_msg.header.stamp = this->now();
          goal_msg.header.frame_id = "map"; // Or "base_link" depending on your math

          // Point data
          goal_msg.point.x = goal_point.pose.position.x;
          goal_msg.point.y = goal_point.pose.position.y;
          goal_msg.point.z = 0.0; 

          goal_pub->publish(goal_msg);
      }
  }
  
  // transformation of the goal point in the car coordinate system
  // goal point in tf notation
  geometry_msgs::msg::PointStamped goal_map_frame;
  goal_map_frame.header.frame_id = "map";
  goal_map_frame.point = goal_point.pose.position;

  geometry_msgs::msg::PointStamped goal_car_frame;


  double lx = -1;
  double ly = -1;
  try
  {    
    // transformation
    tf_buffer_->transform(goal_map_frame, goal_car_frame, "ego_racecar/base_link");
    // extract coordinates
    lx = goal_car_frame.point.x;  // Distance forward from car
    ly = goal_car_frame.point.y;  // Distance left/right from car center

    
  } catch (const tf2::TransformException & ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal transform failed: %s", ex.what());
    return;
  }
  

  // Calc steering angle using pure pursuit geometry using lx and ly

  // (1) calc alpha
  double alpha = std::atan2(ly, lx);

  // (2) calc steering angle
  this->steering_angle = std::atan2( (2*WHEELBASE_LENGTH*sin(alpha)), (this->get_parameter("LOOK_AHEAD_DISTANCE").as_double()));
  this->construct_and_publish_drive_message();
}


void controller::construct_and_publish_drive_message()
{
  // construct message
  ackermann_msgs::msg::AckermannDriveStamped drive_msg;
  //drive_msg.drive.steering_angle = std::clamp(static_cast<float>(this->steering_angle), -0.2f, 0.2f);
  drive_msg.drive.steering_angle = this->steering_angle;
  drive_msg.drive.speed = CAR_SPEED;

  // publish message
  this->drive_speed_pub->publish(drive_msg);
}


void controller::diagnostics_timer_callback()
{
    diagnostic_msgs::msg::DiagnosticArray array_msg;
    array_msg.header.stamp = this->now();

    diagnostic_msgs::msg::DiagnosticStatus status_msg;
    status_msg.name = "safety_node";
    status_msg.hardware_id = "f1tenth_car";


    status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status_msg.message = "Not used";
    

    array_msg.status.push_back(status_msg);

    diagnostics_pub->publish(array_msg);
    std::cout << "successfull ended callback" << std::endl;
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller>());
  rclcpp::shutdown();
  return 0;
}
