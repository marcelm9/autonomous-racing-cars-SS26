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
  
  // variables config
  this->declare_parameter<double>("LOOK_AHEAD_DISTANCE", 1.3);
  this->declare_parameter<double>("CAR_SPEED", 3.0);
  this->current_velocity = 0.0;
  this->target_velocity = 0.0;
  this->steering_angle = 0.0;

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
    return;
  }
  
  // --- DYNAMIC PARAMETERS (Hardcoded logic instead of get_parameter) ---
  const double K_L = 0.32;        // Lookahead gain (seconds). Increase for more stability.
  const double MIN_L = 0.8;       // Minimum lookahead distance (meters)
  const double MAX_L = 4.5;       // Maximum lookahead distance (meters)
  const double MAX_V = 7.0;       // Top speed on straights (m/s)
  const double MIN_V = 3.0;       // Minimum speed in sharp turns (m/s)
  const double MAX_STEER = 0.418; // Max steering limit (approx 24 degrees)
  // ---------------------------------------------------------------------

  geometry_msgs::msg::TransformStamped map_to_base_link;
  try {
      map_to_base_link = this->tf_buffer_->lookupTransform(
          "map", "ego_racecar/base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      return;
  }

  double car_x = map_to_base_link.transform.translation.x;
  double car_y = map_to_base_link.transform.translation.y;

  // 1. Calculate Dynamic Lookahead based on current speed
  // Note: Using the last commanded speed or odom speed if available
  double dynamic_lookahead = std::clamp(this->current_velocity * K_L, MIN_L, MAX_L);

  // 2. Find closest point (Modulo Wrap-around)
  float min_dist = std::numeric_limits<float>::infinity();
  int closest_index = -1;
  size_t path_size = this->current_path.poses.size();

  for (size_t i = 0; i < path_size; ++i) {
      double dist = std::hypot(this->current_path.poses[i].pose.position.x - car_x, 
                               this->current_path.poses[i].pose.position.y - car_y); 
      if (dist < min_dist) {
          min_dist = dist;
          closest_index = i;
      }
  }
  if (closest_index == -1) return;

  // 3. Find goal point using wrap-around
  geometry_msgs::msg::PoseStamped goal_point = this->current_path.poses[closest_index]; 
  for (size_t i = 0; i < path_size; ++i) {
      size_t idx = (closest_index + i) % path_size; // MODULO FIX
      const auto & path_point = this->current_path.poses[idx];
      double dist = std::hypot(path_point.pose.position.x - car_x, 
                               path_point.pose.position.y - car_y);

      if (dist >= dynamic_lookahead) {
          goal_point = path_point;
          break; 
      }
  }

  // 4. Transform goal point to car frame
  geometry_msgs::msg::PointStamped goal_map_frame, goal_car_frame;
  goal_map_frame.header.frame_id = "map";
  goal_map_frame.point = goal_point.pose.position;

  try {    
    tf_buffer_->transform(goal_map_frame, goal_car_frame, "ego_racecar/base_link");
  } catch (const tf2::TransformException & ex) {
    return;
  }

  double lx = goal_car_frame.point.x;  
  double ly = goal_car_frame.point.y;  

  // 5. Calculate steering angle
  double alpha = std::atan2(ly, lx);
  this->steering_angle = std::atan2((2 * WHEELBASE_LENGTH * std::sin(alpha)), dynamic_lookahead);

  // 6. Set Dynamic Velocity based on Steering Angle
  // Higher steering angle = lower velocity
  double steer_ratio = std::abs(this->steering_angle) / MAX_STEER;
  steer_ratio = std::clamp(steer_ratio, 0.0, 1.0);
  
  // Linear interpolation: if steer is 0, speed is MAX_V. If steer is MAX, speed is MIN_V.
  this->target_velocity = MAX_V - (steer_ratio * (MAX_V - MIN_V));

  this->construct_and_publish_drive_message();
}

void controller::construct_and_publish_drive_message()
{
  ackermann_msgs::msg::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = this->now();
  drive_msg.header.frame_id = "base_link";
  
  drive_msg.drive.steering_angle = static_cast<float>(this->steering_angle);
  drive_msg.drive.speed = static_cast<float>(this->target_velocity);

  // Update current_velocity for the next iteration's lookahead calculation
  this->current_velocity = this->target_velocity;

  goal_pub->publish(goal_map_frame);

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
