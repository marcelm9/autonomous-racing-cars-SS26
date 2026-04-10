#include "./../include/reactive/reactive.hpp"

FollowGap::FollowGap() : Node("reactive")
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

/* ---- CALLBACKS ---- */

void FollowGap::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
  this->scan_msg = *scan_msg;

  double angle_increment = scan_msg->angle_increment;
  double angle_min = scan_msg->angle_min;

  std::vector<float> ranges = scan_msg->ranges;

  //// DYNAMIC GAP SEARCH ////
  // look for a gap far away and if none is found reduce the distance until a gap is found

  for (int i = 0; i < this->DYN_THRESHOLD_INCREMENT; i++)
  {
    double threshold = this->MAX_THRESHOLD - (this->MAX_THRESHOLD - this->MIN_THRESHOLD) * i / this->DYN_THRESHOLD_INCREMENT;
    this->find_gaps_by_threshold(ranges, threshold, angle_min, angle_increment, all_gaps, gap_angles);
    if (!gap_angles.empty())
    {
      this->diag_active_threshold_ = threshold; // Save for diagnostics
      break;
    }
  }

  this->all_gaps = all_gaps;
  this->gap_angles = gap_angles;

  //// STEERING WITH WALL AVOIDANCE////
  // Steer to middle of gap

  if (!gap_angles.empty())
  {
    this->goal_angle = gap_angles[gap_angles.size() / 2];
    if (oh_no_im_stuck)
    {
      auto message = std_msgs::msg::String();
      message.data = "gap found again, i owe you my life";
      oh_no_im_stuck_publisher->publish(message);

      oh_no_im_stuck = false;
    }
  }
  else
  {
    if (!oh_no_im_stuck)
    {
      auto message = std_msgs::msg::String();
      message.data = "no gap found, i will die here";
      oh_no_im_stuck_publisher->publish(message);

      oh_no_im_stuck = true;

    }
    this->goal_angle = 0;
  }

  // find closest distance to car on both sides so the steering can be adjusted

  int num_lasers = ranges.size();
  double min_distance = std::numeric_limits<float>::infinity();
  int idx_closest_right;
  int idx_closest_left;

  // right side
  for (int i = 0; i < num_lasers / 2; i++)
  {
    if (ranges[i] < min_distance)
    {
      // find smallest range
      min_distance = ranges[i];
      idx_closest_right = i;
    }
  }

  min_distance = std::numeric_limits<float>::infinity();

  // left side
  for (int i = num_lasers / 2; i < num_lasers; i++)
  {
    if (ranges[i] < min_distance)
    {
      // find smallest range
      min_distance = ranges[i];
      idx_closest_left = i;
    }
  }

  double dist_closest_right = ranges[idx_closest_right];
  double dist_closest_left = ranges[idx_closest_left];

  // Save for diagnostics
  this->diag_dist_closest_right_ = dist_closest_right;
  this->diag_dist_closest_left_ = dist_closest_left;

  // Modify steering angle if close to wall
  double angle_closest_right = idx_closest_right * angle_increment + angle_min;
  double correction_right = 1 / std::pow(dist_closest_right, 2) * (1.2 / std::clamp(angle_closest_right, -1.2, 0.1));

  double angle_closest_left = idx_closest_left * angle_increment + angle_min;
  double correction_left = 1 / std::pow(dist_closest_left, 2) * (1.2 / std::clamp(angle_closest_left, 0.1, 1.2));

  // Save for diagnostics
  this->diag_correction_left_ = correction_left;
  this->diag_correction_right_ = correction_right;

  goal_angle = goal_angle - std::clamp((correction_left + correction_right) / 30, -0.4, 0.4);

  //// SPEED ////
  // Set the speed depending on the open space in front of the car

  min_distance = std::numeric_limits<float>::infinity();
  int idx_closest_front;

  // front side
  for (int i = num_lasers / 2 - 10; i < num_lasers / 2 + 10; i++)
  {
    if (ranges[i] < min_distance)
    {
      // find smallest range
      min_distance = ranges[i];
      idx_closest_front = i;
    }
  }

  // Aggressive speed: full throttle above BRAKE_DIST, linear falloff below
  const double BRAKE_DIST = 2.0; // metres — tune this
  const double MIN_SPEED = 0.5;  // never go slower than this (avoids stalling)

  double speed_factor;
  if (min_distance >= BRAKE_DIST)
  {
    speed_factor = 1.0; // full throttle
  }
  else
  {
    // linear ramp from MIN_SPEED (at 0 m) to MAXDRIVESPEED (at BRAKE_DIST)
    speed_factor = MIN_SPEED / this->MAXDRIVESPEED + (1.0 - MIN_SPEED / this->MAXDRIVESPEED) * (min_distance / BRAKE_DIST);
  }

  // this->diag_dist_closest_front_ = min_distance; // Save for diagnostics

  // double speed_factor = 1 - std::exp(-(this->BRAKE_TUNE) * (min_distance - 0.1)); // offset exp by 0.1 so min value is above 0

  double set_speed = oh_no_im_stuck ? 0 : this->MAXDRIVESPEED * speed_factor;
  this->diag_set_speed_ = set_speed; // Save for diagnostics

  //// PUBLISH ////

  ackermann_msgs::msg::AckermannDriveStamped drive_msg;

  drive_msg.drive.steering_angle = std::clamp(static_cast<float>(this->goal_angle), -0.42f, 0.42f); // should be 0.34
  drive_msg.drive.speed = set_speed;
  drive_speed_pub->publish(drive_msg);

  this->visualize_lidar_ranges();
}

void FollowGap::diag_timer_callback()
{
  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = this->get_clock()->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "FollowGap Internal State";
  status.hardware_id = "ego_racecar";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "Publishing Debug State";

  // Lambda helper to quickly add KeyValue pairs
  auto add_kv = [&status](const std::string &key, double value)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = std::to_string(value);
    status.values.push_back(kv);
  };

  add_kv("1_goal_angle_final", this->goal_angle);
  add_kv("2_target_speed", this->diag_set_speed_);
  add_kv("3_active_threshold", this->diag_active_threshold_);
  add_kv("4_num_gaps_found", static_cast<double>(this->all_gaps.size()));
  add_kv("5_dist_closest_front", this->diag_dist_closest_front_);
  add_kv("6_dist_closest_left", this->diag_dist_closest_left_);
  add_kv("7_dist_closest_right", this->diag_dist_closest_right_);
  add_kv("8_correction_left", this->diag_correction_left_);
  add_kv("9_correction_right", this->diag_correction_right_);

  diag_msg.status.push_back(status);
  diag_pub->publish(diag_msg);
}

void FollowGap::find_gaps_by_threshold(
    const std::vector<float> &ranges,
    double threshold,
    double angle_min,
    double angle_increment,
    std::vector<std::vector<double>> &out_all_gaps,
    std::vector<double> &out_best_gap)
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

  for (int k = static_cast<int>(num_lasers * this->GAP_VIEW_ANGLE_PROCENT); k < static_cast<int>(num_lasers * (1 - this->GAP_VIEW_ANGLE_PROCENT)); k++)
  {
    double angle = k * angle_increment + angle_min;

    if (ranges[k] > threshold)
    {
      // inside a possible gap
      temp_angles.push_back(angle);
      min_count++;
      temp_dist_sum += ranges[k];
    }
    else
    {
      // end of Gap

      if (min_count > this->n_gaphits && temp_dist_sum > best_dist_sum)
      {
        // new best gap
        out_best_gap = temp_angles;
        best_dist_sum = temp_dist_sum;
      }

      // safe as a found gap
      if (!temp_angles.empty() && min_count > this->n_gaphits)
        out_all_gaps.push_back(temp_angles);

      // reset temporary values for next gap
      temp_angles.clear();
      min_count = 0;
      temp_dist_sum = 0.0;
    }
  }

  // edge case: scan ends with a gap still "open"
  if (!temp_angles.empty() && min_count > this->n_gaphits)
  {
    out_all_gaps.push_back(temp_angles);
    if (temp_dist_sum > best_dist_sum)
    {
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
  double g = 1.0;
  double b = 0.0;
  for (int i = 0; i < num_gaps; i++)
  {
    std::vector<double> current_gap = this->all_gaps[i];

    if (current_gap.empty())
    {
      std::cout << "No best gap at time" << rclcpp::Clock().now().seconds() << std::endl;
      return;
    }

    // handle current gap
    double angle_left = current_gap[0];
    double angle_right = current_gap[current_gap.size() - 1];

    int index1 = static_cast<int>((angle_left - angle_min) / angle_increment);
    int index2 = static_cast<int>((angle_right - angle_min) / angle_increment);

    double x1 = std::cos(angle_left) * ranges[index1];
    double y1 = std::sin(angle_left) * ranges[index1];

    double x2 = std::cos(angle_right) * ranges[index2];
    double y2 = std::sin(angle_right) * ranges[index2];

    double temp = g;
    g = b;
    b = temp;
    visualization_msgs::msg::Marker line_left_marker = this->create_line_marker(0, 0, x1, y1, 0.0, g, b, 0.01, 2 * i + 1);
    marker_array.markers.push_back(line_left_marker);
    visualization_msgs::msg::Marker line_right_marker = this->create_line_marker(0, 0, x2, y2, 0.0, g, b, 0.01, 2 * i + 2);
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

  marker.scale.x = width; // line width

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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowGap>());
  rclcpp::shutdown();
  return 0;
};