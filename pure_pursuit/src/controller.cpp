#include <cstdio>
#include "./../include/pure_pursuit/controller.hpp"

controller::controller() : Node("controller")
{
  /* ---- subscribers ---- */
  tf = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, std::bind(&controller::tf_callback, this, std::placeholders::_1));
  path_sub = this->create_subscription<nav_msgs::msg::Path>("/global_path", 10, std::bind(&controller::path_callback, this, std::placeholders::_1));

  /* ---- publishers ---- */
  drive_speed_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);


  

  // publish diagnostics once per second
  
  // variables

}


void controller::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", msg->poses.size());
}
void controller::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received TF message with %zu transforms", msg->transforms.size());
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world pure_pursuit package\n");
  return 0;
}
