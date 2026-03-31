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

  
  //// SAFETY BUBBLE GENERATION ////
  // look for the closest point and set a safety bubble around it
  
  int num_lasers = ranges.size();
  double min_distance = std::numeric_limits<float>::infinity();
  int idx_smallest;

  for (int i = 0; i < num_lasers; i++)  
  {
    if (ranges[i] < min_distance) {
      // find smallest range  
      min_distance = ranges[i];
      idx_smallest = i;
    }
  }

  // Set safety bubble around smallest distance = 0
  int start = std::max(0, idx_smallest - static_cast<int>(this->SAFETYBUBBLE / 2));
  int end = std::min(num_lasers - 1, idx_smallest + static_cast<int>(this->SAFETYBUBBLE / 2));
  this->bubble_index_start = start;
  this->bubble_index_end = end;

  // set distance for points in bubble to 0 exept for the center of the bubble
  for (int j = start; j <= end; j++) {
    if (j == idx_smallest) {
      continue;
    }
    ranges[j] = 0.0;
  }


  //// DYNAMIC GAP SEARCH ////
  // look for a gap far away and if none is found reduce the distance untill a gap is found

  for (int i = 0; i < 6; i++) {
    double threshold = this->MAX_THRESHOLD - (this->MAX_THRESHOLD - this->MIN_THRESHOLD)*i/6;
    this->find_gaps_by_threshold(ranges, threshold, angle_min, angle_increment, all_gaps, gap_angles);
    if (!gap_angles.empty()) {
      // gap found at current threshold
      break;
    }
  }
  
  this->all_gaps = all_gaps;
  this->gap_angles = gap_angles;

 
  
  // Simple implementation, take mid of gap
  if (!gap_angles.empty()) {
      this->goal_angle = gap_angles[gap_angles.size() / 2];
  } else {
      this->goal_angle = 0.0; // Default to straight if no gap found
  } 

  // Modify steering angle if close to wall
  double angle_of_closest = idx_smallest * angle_increment + angle_min;
  double correction = (angle_of_closest/1.5 - 1)*0.43;
  if (ranges[idx_smallest] < this->SAFETY_DIST_WALL) {
    std::cout << "angle of closest   "  << goal_angle<< std::endl;
    goal_angle = goal_angle - 0.2 * static_cast<int>((angle_of_closest/std::abs(angle_of_closest)));
  }
  
  //std::cout << "angle of closest   "  << goal_angle<< std::endl;
  //std::cout << "correction  " << correction<< std::endl;

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
  /*

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

      // Convert local gap index to global laser scan index
      int global_max_idx = static_cast<int>((gap_angles[max_idx] - angle_min) / angle_increment);

      // Now compare global indices safely
      int diff_to_start = std::abs(start - global_max_idx);
      int diff_to_end = std::abs(end - global_max_idx);
      int diff_to_bubble = std::min(diff_to_end, diff_to_start);
      double direction_indicator;
      
      
      //std::cout << "diff to bubbke" << diff_to_bubble << std::endl;
      //std::cout << "diff to bubble" << diff_to_bubble << std::endl;
 

      if (diff_to_bubble > this->TO_BUBBLE_MARGIN) {
        this->goal_angle = gap_angles[max_idx];
        //std::cout << "   ALARM     " << idx_smallest<< std::endl;
      } else {
        if (diff_to_start > diff_to_end) {
          direction_indicator = 1 *( this->TO_BUBBLE_MARGIN - diff_to_end);
          
        } else {
          direction_indicator = -1 * (this->TO_BUBBLE_MARGIN - diff_to_start);
        }
        std::cout << "   goalangle old     " << max_idx << std::endl;
        this->goal_angle = gap_angles[max_idx + direction_indicator];
        std::cout << "   goalangle new     " << max_idx + direction_indicator << std::endl;
      }
      
            
    } else {
      //std::cout << "Gap_angles" << gap_angles << std::endl;
      this->goal_angle = 0.0;
    }
  } else {
    this->goal_angle = 0.0;
  }
  */

  ackermann_msgs::msg::AckermannDriveStamped drive_msg;

  drive_msg.drive.steering_angle = std::clamp(static_cast<float>(this->goal_angle), -0.42f, 0.42f); // should be 0.34
  drive_msg.drive.speed = this->NORMALDRIVESPEED;
  drive_speed_pub->publish(drive_msg);


  this->visualize_lidar_ranges();
}


void FollowGap::find_gaps_by_threshold(
    const std::vector<float>& ranges, 
    double threshold, 
    double angle_min, 
    double angle_increment, 
    std::vector<std::vector<double>>& out_all_gaps, 
    std::vector<double>& out_best_gap)
{

  out_all_gaps.clear();
  out_best_gap.clear();

    // Find possible gaps
  std::vector<double> gap_angles;
  std::vector<std::vector<double>> all_gaps;
  std::vector<double> temp_angles; 
  size_t min_count = 0;

  int num_lasers = ranges.size();
  double temp_dist_sum = 0; 
  double best_dist_sum = 0;

  for (int k = static_cast<int>(num_lasers * 0.25); k < static_cast<int>(num_lasers * 0.75); k++)
  {
    double angle = k * angle_increment + angle_min;
    
    if (ranges[k] > threshold) {
      // inside a possible gap
      temp_angles.push_back(angle);
      min_count++;
      temp_dist_sum += ranges[k];
      
    } else {
      // end of Gap
      
      if (min_count > this->n_gaphits && temp_dist_sum > best_dist_sum) {
        // new best gap
        out_best_gap = temp_angles;
        best_dist_sum = temp_dist_sum;
      }

      // safe as a found gap
      if (!temp_angles.empty() && min_count > this->n_gaphits) out_all_gaps.push_back(temp_angles);
      
      // reset temporary values for next gap
      temp_angles.clear();
      min_count = 0;
      temp_dist_sum = 0.0;
    }
  }

  // edge case: scan ends with a gap still "open"
  if (!temp_angles.empty() && min_count > this->n_gaphits) {
      out_all_gaps.push_back(temp_angles);
      if (temp_dist_sum > best_dist_sum) {
          out_best_gap = temp_angles;
      }
  }
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
