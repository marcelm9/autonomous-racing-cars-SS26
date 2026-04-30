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
      map_to_base_link = this->tf_buffer_->lookupTransform(
          "map", "ego_racecar/base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
      return;
  }

  double car_x = map_to_base_link.transform.translation.x;
  double car_y = map_to_base_link.transform.translation.y;

  // 1. Find closest point (unchanged)
  float min_dist = std::numeric_limits<float>::infinity();
  int closest_index = -1;
  size_t path_size = this->current_path.poses.size();

  for (size_t i = 0; i < path_size; ++i)
  {
      double path_x = this->current_path.poses[i].pose.position.x;
      double path_y = this->current_path.poses[i].pose.position.y;
      // std::hypot is a cleaner/faster way to write sqrt(a^2 + b^2)
      double dist = std::hypot(path_x - car_x, path_y - car_y); 
      
      if (dist < min_dist)
      {
          min_dist = dist;
          closest_index = i;
      }
  }

  if (closest_index == -1) return; // Safety check

  // 2. Find goal point using wrap-around (Modulo Arithmetic)
  double lookahead = this->get_parameter("LOOK_AHEAD_DISTANCE").as_double();
  
  // Default to closest point in case we somehow don't find a lookahead point
  geometry_msgs::msg::PoseStamped goal_point = this->current_path.poses[closest_index]; 

  // Iterate forward along the path starting from the closest index.
  // We check up to 'path_size' to ensure we don't enter an infinite loop.
  for (size_t i = 0; i < path_size; ++i)
  {
      // THE FIX: Modulo operator wraps the index back to 0 if it exceeds the path size.
      size_t idx = (closest_index + i) % path_size; 
      
      const auto & path_point = this->current_path.poses[idx];
      double path_x = path_point.pose.position.x;
      double path_y = path_point.pose.position.y;
      double dist = std::hypot(path_x - car_x, path_y - car_y);

      // Instead of checking for min_diff, standard Pure Pursuit stops at the 
      // FIRST point that is further away than your lookahead distance.
      // This prevents the car from accidentally locking onto track sections 
      // on the other side of a hairpin!
      if (dist >= lookahead)
      {
          goal_point = path_point;
          break; // We found our target, stop searching
      }
  }

  // 3. Publish and transform the goal point
  auto goal_msg = geometry_msgs::msg::PointStamped();
  goal_msg.header.stamp = this->now();
  goal_msg.header.frame_id = "map"; 
  goal_msg.point.x = goal_point.pose.position.x;
  goal_msg.point.y = goal_point.pose.position.y;
  goal_msg.point.z = 0.0; 
  goal_pub->publish(goal_msg);

  geometry_msgs::msg::PointStamped goal_map_frame;
  goal_map_frame.header.frame_id = "map";
  goal_map_frame.point = goal_point.pose.position;
  geometry_msgs::msg::PointStamped goal_car_frame;

  double lx = -1;
  double ly = -1;
  try
  {    
    tf_buffer_->transform(goal_map_frame, goal_car_frame, "ego_racecar/base_link");
    lx = goal_car_frame.point.x;  
    ly = goal_car_frame.point.y;  
  } catch (const tf2::TransformException & ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal transform failed: %s", ex.what());
    return;
  }

  // 4. Calculate steering
  double alpha = std::atan2(ly, lx);
  this->steering_angle = std::atan2( (2 * WHEELBASE_LENGTH * sin(alpha)), lookahead );
  this->construct_and_publish_drive_message();
}


void controller::construct_and_publish_drive_message()
{
  // construct message
  ackermann_msgs::msg::AckermannDriveStamped drive_msg;
  //drive_msg.drive.steering_angle = std::clamp(static_cast<float>(this->steering_angle), -0.2f, 0.2f);
  drive_msg.drive.steering_angle = this->steering_angle;
  drive_msg.drive.speed = this->get_parameter("CAR_SPEED").as_double();

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
