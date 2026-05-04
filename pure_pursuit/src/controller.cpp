#include "./../include/pure_pursuit/controller.hpp"

controller::controller() : Node("controller")
{
  /* ---- subscribers ---- */
  path_sub = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&controller::path_callback, this, std::placeholders::_1));

  /* ---- publishers ---- */
  drive_speed_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
  goal_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/goal_point", 10);
  nearest_point_on_raceline_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/nearest_point_on_raceline", 10);
  car_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/car_pos", 10);

  
  // variables config
  this->declare_parameter<double>("LOOK_AHEAD_DISTANCE", 1.3);
  this->declare_parameter<double>("CAR_SPEED", 3.0);
  this->current_velocity = 0.0;
  this->target_velocity = 0.0;
  this->steering_angle = 0.0;
    // for steering angle based dynamic lookahead
  this->declare_parameter<double>("K_L", 0.32); // Lookahead gain (seconds). Increase for more stability.
  this->declare_parameter<double>("MIN_L", 0.2); // Minimum lookahead distance (meters)
  this->declare_parameter<double>("MAX_L", 3); // Maximum lookahead distance (meters)
  this->declare_parameter<double>("MAX_V", 10.0); // Top speed on straights (m/s)
  this->declare_parameter<double>("MIN_V", 1.0); // Minimum speed in sharp turns (m/s)
  this->declare_parameter<double>("MAX_STEER", 0.218); // Max steering limit (approx 24 degrees)

    // for lockahead angle based dynamic lookahead
  this->declare_parameter<double>("MAX_L_ANGLE", 0.3); 
  this->declare_parameter<double>("L_INCREMENT", 0.2);

  // timers
  pure_pursuit_timer = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&controller::pure_pursuit_timer_callback, this)
  );


  // Diagnostics 
  diagnostics_timer = this->create_wall_timer(
    std::chrono::milliseconds(10), 
    std::bind(&controller::diagnostics_timer_callback, this)
  ); 
  // Open the log file
    log_file.open("/arc2026/ws/src/your_code/pure_pursuit/position_log.csv", std::ios::out | std::ios::app);
    if (log_file.is_open()) {
        log_file << "timestamp,x,y\n"; // Write header
    }

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

void controller::pure_pursuit_timer_callback()
{
  if (!this->path_received_) {
    RCLCPP_INFO(this->get_logger(), "No path received");
    return;
  }

  geometry_msgs::msg::TransformStamped map_to_base_link;

  try {
      map_to_base_link = this->tf_buffer_->lookupTransform(
          "map", this->base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "map", this->base_frame, ex.what());
      return;
  }

  double car_x = map_to_base_link.transform.translation.x;
  double car_y = map_to_base_link.transform.translation.y;
  this->latest_car_x = car_x;
  this->latest_car_y = car_y;

  // Calculate Dynamic Lookahead based on current speed
  double dynamic_lookahead = std::clamp(this->current_velocity * this->get_parameter("K_L").as_double(), this->get_parameter("MIN_L").as_double(), this->get_parameter("MAX_L").as_double());
  //double dynamic_lookahead = this->get_parameter("LOOK_AHEAD_DISTANCE").as_double(); // For fixed lookahead testing
  // Find closest point (Modulo Wrap-around)
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
  if (closest_index == -1) {
      RCLCPP_INFO(this->get_logger(), "Could not find closest point on path");
      return;
  } 
  // Find goal point using wrap-around
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

  // Transform goal point to car frame
  geometry_msgs::msg::PointStamped goal_map_frame, goal_car_frame;
  goal_map_frame.header.frame_id = "map";
  goal_map_frame.point = goal_point.pose.position;

  geometry_msgs::msg::PointStamped nearest_point_on_raceline;
  nearest_point_on_raceline.header.frame_id = "map";
  nearest_point_on_raceline.point = this->current_path.poses[closest_index].pose.position;

  geometry_msgs::msg::PointStamped car_pos;
  car_pos.header.frame_id = "map";
  car_pos.header.stamp = this->get_clock()->now();
  car_pos.point.x = car_x;
  car_pos.point.y = car_y;
  car_pos.point.z = 0.0;
 
  try {    
    tf_buffer_->transform(goal_map_frame, goal_car_frame, this->base_frame);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform 'goal_map_frame' to 'particle_filter_base_frame': %s", ex.what());
    return;
  }

  double lx = goal_car_frame.point.x;  
  double ly = goal_car_frame.point.y;  

  // Calculate steering angle
  double alpha = std::atan2(ly, lx);  
  this->steering_angle = std::atan2((2 * WHEELBASE_LENGTH * std::sin(alpha)), dynamic_lookahead)/2.0; // Dividing by 2 because of mismatch of steering angle


  // Set Dynamic Velocity based on Steering Angle
  // Higher steering angle = lower velocity
  double steer_ratio = std::abs(this->steering_angle) / this->get_parameter("MAX_STEER").as_double();
  steer_ratio = std::clamp(steer_ratio, 0.0, 1.0);
  
  // Linear interpolation: if steer is 0, speed is MAX_V. If steer is MAX, speed is MIN_V.
  this->target_velocity = this->get_parameter("MAX_V").as_double() - (steer_ratio * (this->get_parameter("MAX_V").as_double() - this->get_parameter("MIN_V").as_double()));

  goal_pub->publish(goal_map_frame);
  nearest_point_on_raceline_pub->publish(nearest_point_on_raceline);
  car_pub->publish(car_pos);

  this->construct_and_publish_drive_message();
}

void controller::MikaTim_pure_pursuit_timer_callback()
{
  if (!this->path_received_) {
    RCLCPP_INFO(this->get_logger(), "No path received");
    return;
  }

  geometry_msgs::msg::TransformStamped map_to_base_link;

  try {
      map_to_base_link = this->tf_buffer_->lookupTransform(
          "map", this->base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "map", this->base_frame, ex.what());
      return;
  }

  double car_x = map_to_base_link.transform.translation.x;
  double car_y = map_to_base_link.transform.translation.y;
  this->latest_car_x = car_x;
  this->latest_car_y = car_y;

  // Increase lockahead until angle to lockahead is higher than max angle
  double dynamic_lookahead = this->get_parameter("MIN_L").as_double();
  

  // Find closest point 
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
  if (closest_index == -1) {
      RCLCPP_INFO(this->get_logger(), "Could not find closest point on path");
      return;
  } 
  // Find goal point using wrap-around
  geometry_msgs::msg::PoseStamped goal_point = this->current_path.poses[closest_index]; 
  for (size_t i = 0; i < path_size; ++i) {
      size_t idx = (closest_index + i) % path_size; 
      const auto & path_point = this->current_path.poses[idx];
      double dist = std::hypot(path_point.pose.position.x - car_x, 
                               path_point.pose.position.y - car_y);

      if (dist >= dynamic_lookahead) {
          float lockahead_angle = std::atan2(path_point.pose.position.y - car_y, path_point.pose.position.x - car_x);
          if (std::abs(lockahead_angle) > this->get_parameter("MAX_L_ANGLE").as_double()) {
              goal_point = path_point;
              break;
          }
          else {
              dynamic_lookahead += this->get_parameter("L_INCREMENT").as_double(); // Increment lookahead and check again
              if (dynamic_lookahead > this->get_parameter("MAX_L").as_double()) {
                  goal_point = path_point; // If we exceed max lookahead, just take this point
                  break;
              }
          }
          
      }
  }

  // Transform goal point to car frame
  geometry_msgs::msg::PointStamped goal_map_frame, goal_car_frame;
  goal_map_frame.header.frame_id = "map";
  goal_map_frame.point = goal_point.pose.position;

  geometry_msgs::msg::PointStamped nearest_point_on_raceline;
  nearest_point_on_raceline.header.frame_id = "map";
  nearest_point_on_raceline.point = this->current_path.poses[closest_index].pose.position;

  geometry_msgs::msg::PointStamped car_pos;
  car_pos.header.frame_id = "map";
  car_pos.header.stamp = this->get_clock()->now();
  car_pos.point.x = car_x;
  car_pos.point.y = car_y;
  car_pos.point.z = 0.0;
 
  // particle_filter_base_frame
  try {
    tf_buffer_->transform(goal_map_frame, goal_car_frame, this->base_frame);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform 'goal_map_frame' to 'particle_filter_base_frame': %s", ex.what());
    return;
  }

  double lx = goal_car_frame.point.x;  
  double ly = goal_car_frame.point.y;  

  // Calculate steering angle
  double alpha = std::atan2(ly, lx);
  this->steering_angle = std::atan2((2 * WHEELBASE_LENGTH * std::sin(alpha)), dynamic_lookahead);

  // Set Dynamic Velocity based on Steering Angle
  // Higher steering angle = lower velocity
  double steer_ratio = std::abs(this->steering_angle) / this->get_parameter("MAX_STEER").as_double();
  steer_ratio = std::clamp(steer_ratio, 0.0, 1.0);
  
  // Linear interpolation: if steer is 0, speed is MAX_V. If steer is MAX, speed is MIN_V.
  this->target_velocity = this->get_parameter("MAX_V").as_double() - (steer_ratio * (this->get_parameter("MAX_V").as_double() - this->get_parameter("MIN_V").as_double()));

  goal_pub->publish(goal_map_frame);
  nearest_point_on_raceline_pub->publish(nearest_point_on_raceline);
  car_pub->publish(car_pos);

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

  this->drive_speed_pub->publish(drive_msg);
}


void controller::diagnostics_timer_callback()
{
    if (log_file.is_open()) {
        // Log current time (ROS 2 time), X, and Y
        log_file << this->now().seconds() << "," 
                 << this->latest_car_x << "," 
                 << this->latest_car_y << "\n";
        
        // Flush to ensure data is written if the node crashes
        log_file.flush();
    }

}

controller::~controller() {
    if (log_file.is_open()) {
        log_file.close();
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller>());
  rclcpp::shutdown();
  return 0;
}
