#include "./../include/reactive/reactive_node.hpp"

FollowGap::FollowGap() : Node("reactive_node")
{
  /* ---- subscribers ---- */
  scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&FollowGap::scan_callback, this, std::placeholders::_1));
  
  
  /* ---- publishers ---- */
  drive_speed_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_out", 10);

  // variables


}

/* ---- CALLBACKS ---- */

void FollowGap::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
{
  this->scan_msg = *scan_msg;

  double angle_increment = scan_msg->angle_increment;
  double angle_min = scan_msg->angle_min;
  double angle_max = scan_msg->angle_max;

  std::vector<float> ranges = scan_msg->ranges;

  // iterate through all lasers and project
  int num_lasers = ranges.size();
  double min_distance = std::numeric_limits<float>::infinity();
  double current_angle;

  int idx_smallest;

  for (int i = 0; i < num_lasers; i++)  
  {
    double angle = i * angle_increment + angle_min;

    if (ranges[i] < min_distance) {
        min_distance = ranges[i];
        current_angle = angle;
        idx_smallest = i;
    }
  }

  // Set safety bubble around smallest distance = 0
  for (int j = idx_smallest - this->SAFETYBUBBLE/2;
    j < idx_smallest + this->SAFETYBUBBLE/2;
    j ++)
  {
    ranges[j] = 0;
  }

  double gap_left = std::abs(angle_max - (current_angle + this->SAFETYBUBBLE/2 * angle_increment));

  double gap_right = std::abs(angle_min - (current_angle - this->SAFETYBUBBLE/2 * angle_increment));

  double goal_angle;
  
  if (gap_left < gap_right)
  {
    goal_angle = angle_min + gap_right/2;
  } else
  {
    goal_angle = angle_min + gap_right/2;
  }

  ackermann_msgs::msg::AckermannDriveStamped drive_msg;

  drive_msg.drive.speed = this->NORMALDRIVESPEED;
  drive_msg.drive.steering_angle = goal_angle;
  drive_speed_pub->publish(drive_msg);  
}


void visualize_lidar_ranges()
    {
      std::vector<float> ranges = this->scan_msg.ranges();
      int num_lasers = ranges.size();
      
      double angle_increment = this->scan_msg.angle_increment;
      double angle_min = this->scan_msg.angle_min;
      double angle_max = this->scan_msg.angle_max;

      visualization_msgs::msg::MarkerArray marker_array;
      for (i = 0; i < num_lasers; i++) {
        double angle = i * angle_increment + angle_min;
        // create message
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "ego_racecar/laser";
        marker.header.stamp = rclcpp::Clock().now();

        marker.ns = "basic_shapes";
        marker.id = 0;

        marker.type = visualization_msgs::msg::Marker::SPHERE;

        marker.action = visualization_msgs::msg::Marker::ADD;


        // project marker position
        double x = std::cos(angle) * ranges[i];
        double y = std::sin(angle) * ranges[i];

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

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker_arra.push_back(marker);
      }

        ebrake_vis_pub->publish(marker);
    }

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowGap>());
    rclcpp::shutdown();
    return 0;
};
