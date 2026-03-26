#include "./../include/reactive/reactive_node.hpp"

FollowGap::FollowGap() : Node("reactive_node")
{
  /* ---- subscribers ---- */
  scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&FollowGap::scan_callback, this, std::placeholders::_1));
  
  
  /* ---- publishers ---- */
  drive_speed_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_out", 10);

  gap_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/gap_vis", 10);


  // variables


}

/* ---- CALLBACKS ---- */

void FollowGap::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
{
  this->scan_msg = *scan_msg;

  double angle_increment = scan_msg->angle_increment;
  double angle_min = scan_msg->angle_min;

  std::vector<float> ranges = scan_msg->ranges;

  // iterate through all lasers and project
  int num_lasers = ranges.size();
  double min_distance = std::numeric_limits<float>::infinity();

  int idx_smallest;

  
  for (int i = 0; i < num_lasers; i++)  
  {
    if (ranges[i] < min_distance) {
        min_distance = ranges[i];
        idx_smallest = i;
    }
  }

  // Set safety bubble around smallest distance = 0
  int start = std::max(0, idx_smallest - static_cast<int>(this->SAFETYBUBBLE / 2));
  int end = std::min(num_lasers - 1, idx_smallest + static_cast<int>(this->SAFETYBUBBLE / 2));

  for (int j = start; j <= end; j++) {
      ranges[j] = 0.0;
  }

  // Find possible gaps
  std::vector<double> gap_angles;
  std::vector<double> temp_angles; 
  int min_count = 0;
  for (int k = static_cast<int>(num_lasers * 0.25); k < num_lasers * 0.75; k++)
  {
    double angle = k * angle_increment + angle_min;
    if (ranges[k] > this->THRESHOLD)
    {
      // possible gap
      temp_angles.push_back(angle);
      min_count++;
      
    } else 
    {
      // end of Gap
      if (min_count > this-> n_gabhits && temp_angles.size() > gap_angles.size()){
        // new best gap
        gap_angles = temp_angles;
      }
      min_count = 0;
      temp_angles.clear();
    }
  }
  if (temp_angles.size() > gap_angles.size() && temp_angles.size() > this->n_gabhits) {
    gap_angles = temp_angles;
  }

  // Simple implementation, take mid of gap
  if (!gap_angles.empty()) {
      this->goal_angle = gap_angles[gap_angles.size() / 2];
  } else {
      this->goal_angle = 0.0; // Default to straight if no gap found
  }
  
  //std::cout << "Goal angle" << goal_angle<< std::endl;

  ackermann_msgs::msg::AckermannDriveStamped drive_msg;

  drive_msg.drive.steering_angle = std::clamp(static_cast<float>(this->goal_angle), -0.4189f, 0.4189f);
  drive_msg.drive.speed = this->NORMALDRIVESPEED;
  drive_speed_pub->publish(drive_msg);


  this->visualize_lidar_ranges();
}


void FollowGap::visualize_lidar_ranges()
    {
      std::vector<float> ranges = this->scan_msg.ranges;
      int num_lasers = ranges.size();
      
      double angle_increment = this->scan_msg.angle_increment;
      double angle_min = this->scan_msg.angle_min;
      double angle_max = this->scan_msg.angle_max;

      visualization_msgs::msg::MarkerArray marker_array;
      for (int i = 0; i < num_lasers; i++) {

        double angle = i * angle_increment + angle_min;

        // project marker position
        double x = std::cos(angle) * ranges[i];
        double y = std::sin(angle) * ranges[i];

        visualization_msgs::msg::Marker marker = this->create_marker(x, y, 0.0, 1.0, 0.0, i);

        marker_array.markers.push_back(marker);
      }

      // project goal angle
      double x = std::cos(this->goal_angle) * 1;
      double y = std::sin(this->goal_angle) * 1;

      visualization_msgs::msg::Marker marker = this->create_marker(x, y, 1.0, 0.0, 0.0, num_lasers);
      marker_array.markers.push_back(marker);

      // publish array
      gap_publisher->publish(marker_array);
    }



  visualization_msgs::msg::Marker FollowGap::create_marker(double x, double y, double r, double g, double b, int id)
  {
        // create message
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "ego_racecar/laser";
        marker.header.stamp = rclcpp::Clock().now();

        marker.ns = "basic_shapes";
        marker.id = id;

        marker.type = visualization_msgs::msg::Marker::SPHERE;

        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        return marker;
  }
    


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowGap>());
    rclcpp::shutdown();
    return 0;
};
