#include "./../include/reactive/reactive_node.hpp"

FollowGap::FollowGap() : Node("reactive_node")
{
  /* ---- subscribers ---- */
  scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&FollowGap::scan_callback, this, std::placeholders::_1));
  
  
  /* ---- publishers ---- */
  drive_speed_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_out", 10);

  gap_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/gap_vis", 1);


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

  this->bubble_index_start = start;
  this->bubble_index_end = end;

  for (int j = start; j <= end; j++) {
      ranges[j] = 0.0;
  }

  // Find possible gaps
  std::vector<double> gap_angles;
  std::vector<std::vector<double>> all_gaps;
  std::vector<double> temp_angles; 
  size_t min_count = 0;

  double sum_distance = 0.0;
  int valid_count = 0;

  for (int i = 0; i < num_lasers; i++) {
      if (std::isfinite(ranges[i])) {
          sum_distance += ranges[i];
          valid_count++;
      }
  }

  double mean_distance = 0.0;
  if (valid_count > 0) {
      mean_distance = sum_distance / valid_count;
  }
  //std::cout << mean_distance << std::endl;
  this->THRESHOLD = mean_distance;

  for (int k = static_cast<int>(num_lasers * 0.2); k < static_cast<int>(num_lasers * 0.8); k++)
  {
    double angle = k * angle_increment + angle_min;
    if (ranges[k] > this->THRESHOLD)
    {
      // possible gap
      temp_angles.push_back(angle);
      min_count++;
      
    } else {
      // end of Gap
      if (min_count > this->n_gaphits && temp_angles.size() > gap_angles.size()) {
        // new best gap
        gap_angles = temp_angles;
      }

      if (!temp_angles.empty() && min_count > this->n_gaphits) all_gaps.push_back(temp_angles);
      temp_angles.clear();
      min_count = 0;
    }
  }
  if (!temp_angles.empty() && min_count > this->n_gaphits) {
    all_gaps.push_back(temp_angles);
  }
  if (temp_angles.size() > gap_angles.size() && temp_angles.size() > this->n_gaphits) {
      gap_angles = temp_angles;
  }

  this->all_gaps = all_gaps;
  this->gap_angles = gap_angles;

 
  /*
  // Simple implementation, take mid of gap
  if (!gap_angles.empty()) {
      this->goal_angle = gap_angles[gap_angles.size() / 2];
  } else {
      this->goal_angle = 0.0; // Default to straight if no gap found
  } 
  */
  //std::cout << "Goal angle" << goal_angle<< std::endl;
/*
  
  // Take mean ray as goal
  if (!gap_angles.empty()){
    std::vector<double> gap_lengths;
    for (double angle : gap_angles)
    {
      int ray_idx = static_cast<int>((angle - angle_min) / angle_increment);
      if (ray_idx>= 0 && ray_idx < static_cast<int>(ranges.size())) {
        gap_lengths.push_back(static_cast<double>(ranges[ray_idx]));
      }
    }
    if (!gap_lengths.empty()) {
      double sum = std::accumulate(gap_lengths.begin(), gap_lengths.end(), 0.0);
      double mean = sum / static_cast<double>(gap_lengths.size());
      double min_distance_mean = std::numeric_limits<double>::infinity();
      int best_gap_idx = 0;
      for (size_t i = 0; i < gap_lengths.size(); i++)
      {
        double distance = std::abs(gap_lengths[i] - mean);
        if (distance < min_distance_mean)
        {
          min_distance_mean = distance;
          best_gap_idx = static_cast<int>(i);
        }
      }
      this->goal_angle = gap_angles[best_gap_idx];
    } else {
      // std::cout << "Gap_angles" << gap_angles << std::endl;
      this->goal_angle = 0.0;
    }
  } else {
    this->goal_angle = 0.0;
  }
  */

  // May ray as gives direction
  

  if (!gap_angles.empty()){
    std::vector<double> gap_lengths;
    for (double angle : gap_angles) {
      int ray_idx = static_cast<int>((angle - angle_min) / angle_increment);
      if (ray_idx>= 0 && ray_idx < static_cast<int>(ranges.size())) {
        gap_lengths.push_back(static_cast<double>(ranges[ray_idx]));
      }
    }
    if (!gap_lengths.empty()) {
      // find largest distance
      auto it = std::max_element(gap_lengths.begin(), gap_lengths.end());
      int max_idx = std::distance(gap_lengths.begin(), it);
      this->goal_angle = gap_angles[max_idx];      
    } else {
      // std::cout << "Gap_angles" << gap_angles << std::endl;
      this->goal_angle = 0.0;
    }
  } else {
    this->goal_angle = 0.0;
  }
  

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

      // add clear marker
      visualization_msgs::msg::Marker clear_marker;
      clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(clear_marker);


      // visualize saftey bubble
      double bouble_angel_start = angle_min + this->bubble_index_start * angle_increment;
      double bouble_angel_end = angle_min + this->bubble_index_end * angle_increment;

      double bouble_x1 = std::cos(bouble_angel_start) * ranges[this->bubble_index_start];
      double bouble_y1 = std::sin(bouble_angel_start) * ranges[this->bubble_index_start];
      double bouble_x2 = std::cos(bouble_angel_end) * ranges[this->bubble_index_end];
      double bouble_y2 = std::sin(bouble_angel_end) * ranges[this->bubble_index_end];

      visualization_msgs::msg::Marker bubble_marker = this->create_line_marker(bouble_x1, bouble_y1, bouble_x2, bouble_y2, 1.0, 0.0, 0.0, 0.01, 0);
      marker_array.markers.push_back(bubble_marker);

      // viz gaps
      int num_gaps = this->all_gaps.size();
      std::cout << num_gaps << std::endl;
      double g = 1.0;
      double b = 0.0;
      for (int i = 0; i < num_gaps; i++) {
        std::vector<double> current_gap = this->all_gaps[i];

        if (current_gap.empty()) {
          std::cout << "No best gap at time" << rclcpp::Clock().now().seconds() << std::endl;
        return;
      }

        // handle current gap
        double angle_left = current_gap[0];
        double angle_right = current_gap[current_gap.size()-1];

        int index1 = static_cast<int>((angle_left - angle_min) / angle_increment);
        int index2 = static_cast<int>((angle_right - angle_min) / angle_increment);

        double x1 = std::cos(angle_left) * ranges[index1];
        double y1 = std::sin(angle_left) * ranges[index1];

        double x2 = std::cos(angle_right) * ranges[index2];
        double y2 = std::sin(angle_right) * ranges[index2];

        double temp = g;;
        g = b;
        b = temp;
        visualization_msgs::msg::Marker line_left_marker = this->create_line_marker(0, 0, x1, y1, 0.0, g, b, 0.01, 2*i+1);
        marker_array.markers.push_back(line_left_marker);
        visualization_msgs::msg::Marker line_right_marker = this->create_line_marker(0, 0, x2, y2, 0.0, g, b, 0.01, 2*i+2);
        marker_array.markers.push_back(line_right_marker);


      }

      // project goal angle
      double x = std::cos(this->goal_angle) * 1;
      double y = std::sin(this->goal_angle) * 1;

      visualization_msgs::msg::Marker marker = this->create_marker(x, y, 1.0, 0.0, 0.0, 3.0, num_lasers);
      marker_array.markers.push_back(marker);

      // publish array
      gap_publisher->publish(marker_array);
    }



  visualization_msgs::msg::Marker FollowGap::create_marker(double x, double y, double r, double g, double b, double size_factor, int id)
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

        marker.scale.x = 0.03 * size_factor;
        marker.scale.y = 0.03 * size_factor;
        marker.scale.z = 0.03 * size_factor;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        return marker;
  }

  visualization_msgs::msg::Marker FollowGap::create_line_marker(double x1, double y1, double x2, double y2, double r, double g, double b, double width, int id)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "ego_racecar/laser";
    marker.header.stamp = this->get_clock()->now();

    marker.ns = "lines";
    marker.id = id;

    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = width;   // line width

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    geometry_msgs::msg::Point p1;
    p1.x = x1;
    p1.y = y1;
    p1.z = 0.0;

    geometry_msgs::msg::Point p2;
    p2.x = x2;
    p2.y = y2;
    p2.z = 0.0;

    marker.points.push_back(p1);
    marker.points.push_back(p2);

    return marker;
}
    


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowGap>());
    rclcpp::shutdown();
    return 0;
};
