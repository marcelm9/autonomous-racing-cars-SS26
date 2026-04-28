
#include <cstdio>
#include "./../include/pure_pursuit/controller.hpp"

controller::controller() : Node("controller")
{
  /* ---- subscribers ---- */
  tf = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, std::bind(&controller::tf_callback, this, std::placeholders::_1));
  path_sub = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&controller::path_callback, this, std::placeholders::_1));

  /* ---- publishers ---- */
  drive_speed_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

  std::cout << "in constructor" << std::endl;  

  // publish diagnostics once per second
  
  // variables

  // timers
  pure_persuit_timer = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&controller::pure_persuit_timer_callback, this)
  );

}


void controller::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  //RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", msg->poses.size());

  this->current_path = *msg;
}

void controller::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  //RCLCPP_INFO(this->get_logger(), "Received TF message with %zu transforms", msg->transforms.size());
  this->tf_tree = *msg;
}

void controller::pure_persuit_timer_callback()
{
  std::cout << "in timer callback" << std::endl;
  for (const auto & t : this->tf_tree.transforms)
        {
            if (t.header.frame_id == "map" &&
                t.child_frame_id == "ego_racecar/base_link")
            {
                double x = t.transform.translation.x;
                double y = t.transform.translation.y;
                double z = t.transform.translation.z;

                std::cout << "x: " << x << "y: " << y << "z: " << z << std::endl;
            }
        }
  
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller>());
  rclcpp::shutdown();
  return 0;
}
