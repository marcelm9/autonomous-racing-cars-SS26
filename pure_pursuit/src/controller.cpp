#include <cstdio>
#include "./../include/pure_pursuit/controller.hpp"

controller::controller() : Node("controller")
{
  /* ---- subscribers ---- */
  scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&controller::scan_callback, this, std::placeholders::_1));
  tf = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, std::bind(&controller::tf_callback, this, std::placeholders::_1));
  path_sub = this->create_subscription<nav_msgs::msg::Path>("/global_path", 10, std::bind(&controller::path_callback, this, std::placeholders::_1));

  /* ---- publishers ---- */
  drive_speed_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);


  

  // publish diagnostics once per second
  diag_timer = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&FollowGap::diag_timer_callback, this));

  // variables

}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world pure_pursuit package\n");
  return 0;
}
