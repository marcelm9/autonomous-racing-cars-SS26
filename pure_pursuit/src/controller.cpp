#include <cstdio>
#include "./../include/pure_pursuit/controller.hpp"

controller::controller() : Node("controller")
{
  /* ---- subscribers ---- */
  scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&FollowGap::scan_callback, this, std::placeholders::_1));

  /* ---- publishers ---- */
  drive_speed_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
  gap_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/gap_vis", 1);
  diag_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
  oh_no_im_stuck_publisher = this->create_publisher<std_msgs::msg::String>("/oh_no_im_stuck", 1);

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
