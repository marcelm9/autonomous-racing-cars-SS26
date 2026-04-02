#include "./../include/slow_crash/slow_crash.hpp"


SlowCrash::SlowCrash() : Node("SlowCrash")
{
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), 
        std::bind(&SlowCrash::timer_callback, this)
    );

    /* ---- publishers ---- */
    drive_speed = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_in", 10);

}

void SlowCrash::timer_callback()
{
  ackermann_msgs::msg::AckermannDriveStamped drive_msg;
  drive_msg.drive.speed = 0.3;
  drive_speed->publish(drive_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlowCrash>());
  rclcpp::shutdown();
  return 0;
};
