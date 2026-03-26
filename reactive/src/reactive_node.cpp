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


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowGap>());
    rclcpp::shutdown();
    return 0;
};
