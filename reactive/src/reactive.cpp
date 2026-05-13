#include "./../include/reactive/reactive.hpp"

FollowGap::FollowGap() : Node("reactive")
{
  /* ---- subscribers ---- */
  scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&FollowGap::scan_callback, this, std::placeholders::_1));
  path_sub = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&FollowGap::path_callback, this, std::placeholders::_1));
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&FollowGap::odom_callback, this, std::placeholders::_1));

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
  this->declare_parameter<double>("max_speed", 3.0);
  this->declare_parameter<int>("reac_n_extend", 15);
  this->declare_parameter<double>("reac_step_depth", 0.1);
}

/* ---- CALLBACKS ---- */

void FollowGap::path_callback(const nav_msgs::msg::Path::ConstSharedPtr path_msg)
{
  this->path_msg = *path_msg;
}

void FollowGap::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
  this->odom_msg = *odom_msg;
}

void FollowGap::parameter_callback(const std::vector<std::string> &changed_parameters)
{
    for (const auto & param_name : changed_parameters) {
        if (param_name == "max_speed") {
            this->get_parameter("max_speed", this->MAXDRIVESPEED);
        }
        if (param_name == "reac_n_extend") {
            this->get_parameter("reac_n_extend", this->n_extend);
        }
        if (param_name == "reac_step_depth") {
            this->get_parameter("reac_step_depth", this->step_depth);
        }
    }
}

void FollowGap::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
  this->scan_msg = *scan_msg;

  this->get_parameter("max_speed", this->MAXDRIVESPEED);
  this->get_parameter("reac_n_extend", this->n_extend);
  this->get_parameter("reac_step_depth", this->step_depth);

  double angle_increment = scan_msg->angle_increment;
  double angle_min = scan_msg->angle_min;

  std::vector<float> ranges = scan_msg->ranges;

  //// DISPARITY EXTENSION ////

  std::vector<float> extended_ranges;
  this->disparity_extender(ranges, this->n_extend, this->step_depth, extended_ranges);

  //// DYNAMIC GAP SEARCH ////
  // look for a gap far away and if none is found reduce the distance until a gap is found

  for (int i = 0; i < this->DYN_THRESHOLD_INCREMENT; i++)
  {
    double threshold = this->MAX_THRESHOLD - (this->MAX_THRESHOLD - this->MIN_THRESHOLD) * i / this->DYN_THRESHOLD_INCREMENT;
    this->find_gaps_by_threshold(extended_ranges, threshold, angle_min, angle_increment, all_gaps, gap_angles);
    if (!gap_angles.empty())
    {
      this->diag_active_threshold = threshold; // Save for diagnostics
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



  int num_lasers = ranges.size();
  double min_distance = std::numeric_limits<float>::infinity();
  int idx_closest_right = -1;
  int idx_closest_left = -1;

  const double MAX_RELEVANT_ANGLE = M_PI_2/1.4; // ignore walls behind the car
  const double REAR_CLEARANCE = 0.2;      // ignore obstacles too far behind for a rear tire hit

  // check if a ray is relevant for wall avoidance (in front of the car and valid)
  auto is_relevant_ray = [&](int i) {
    double angle = i * angle_increment + angle_min;
    if (std::abs(angle) > MAX_RELEVANT_ANGLE)
      return false; // ignore rays behind the car
    double distance = static_cast<double>(ranges[i]);
    if (!std::isfinite(distance) || distance <= 0.0)
      return false; // ignore invalid measurements
    double x = std::cos(angle) * distance;
    return x >= -REAR_CLEARANCE; // ignore obstacles too far behind for a rear tire hit
  };

  int gap_start_idx = -1;
  int gap_end_idx = -1;

  if (!gap_angles.empty())
  {
    gap_start_idx = static_cast<int>((gap_angles.front() - angle_min) / angle_increment);
    gap_end_idx = static_cast<int>((gap_angles.back() - angle_min) / angle_increment);
  }
  // only consider rays outside of the gap if a valid gap was found
  bool use_gap_bounds = (gap_start_idx >= 0 && gap_end_idx >= 0 && gap_start_idx < gap_end_idx); 
  int right_start = 0;
  int right_end = use_gap_bounds ? gap_start_idx - 1 : num_lasers / 2 - 1;
  int left_start = use_gap_bounds ? gap_end_idx + 1 : num_lasers / 2;
  int left_end = num_lasers - 1;

  // cumulative repulsive force from the right wall
  double correction_right = 0.0;
  for (int i = right_start; i <= right_end; i++)
  {
    if (!is_relevant_ray(i))
      continue;
    
    // track closest point for geometric width constraint
    if (ranges[i] < min_distance)
    {
      min_distance = ranges[i];
      idx_closest_right = i;
    }

    // Cumulative steering force from the right wall, using an elliptical Gaussian to weight points more if they are in front of the car
    double angle = i * angle_increment + angle_min;
    double dist = static_cast<double>(ranges[i]);
    double ellipse = (this->SIGMA_FRONT * this->SIGMA_SIDE) /
        std::sqrt(std::pow(this->SIGMA_SIDE * std::cos(angle), 2) + std::pow(this->SIGMA_FRONT * std::sin(angle), 2));
    
    correction_right += std::exp(-(std::pow(dist, 2) / (2 * ellipse * ellipse)));
  }
  // Scale by angle_increment to make the integral independent of LIDAR resolution
  correction_right *= (angle_increment * this->SIGMA_AMPLITUDE);

  double min_distance_left = std::numeric_limits<float>::infinity();
  double correction_left = 0.0;

  // left side (left of the selected gap)
  for (int i = left_start; i <= left_end; i++)
  {
    if (!is_relevant_ray(i))
      continue;
    
    // track closest point for geometric width constraint
    if (ranges[i] < min_distance_left)
    {
      min_distance_left = ranges[i];
      idx_closest_left = i;
    }

    // Cumulative steering force from the left wall, using an elliptical Gaussian to weight points more if they are in front of the car
    double angle = i * angle_increment + angle_min;
    double dist = static_cast<double>(ranges[i]);
    double ellipse = (this->SIGMA_FRONT * this->SIGMA_SIDE) /
        std::sqrt(std::pow(this->SIGMA_SIDE * std::cos(angle), 2) + std::pow(this->SIGMA_FRONT * std::sin(angle), 2));
    
    correction_left += std::exp(-(std::pow(dist, 2) / (2 * ellipse * ellipse)));
  }
  correction_left *= (angle_increment * this->SIGMA_AMPLITUDE);

  // Update closest point state for visualization/diagnostics
  if (idx_closest_right >= 0) {
    this->closest_idx_right = idx_closest_right;
    this->closest_angle_right = idx_closest_right * angle_increment + angle_min;
    this->closest_dist_right = ranges[idx_closest_right];
    this->diag_dist_closest_right = this->closest_dist_right;
  } else {
    this->closest_idx_right = -1;
    this->diag_dist_closest_right = this->MAX_DIST_STEER;
  }

  if (idx_closest_left >= 0) {
    this->closest_idx_left = idx_closest_left;
    this->closest_angle_left = idx_closest_left * angle_increment + angle_min;
    this->closest_dist_left = ranges[idx_closest_left];
    this->diag_dist_closest_left = this->closest_dist_left;
  } else {
    this->closest_idx_left = -1;
    this->diag_dist_closest_left = this->MAX_DIST_STEER;
  }

  this->diag_correction_left = correction_left;
  this->diag_correction_right = correction_right;

  this->goal_angle_before_width_clamp = this->goal_angle; // Save for visualization

  // Geometric safety: ensure we steer far enough to clear the car's width
  const double half_car_width = 6*0.2032 / 2.0; // approx 0.4m

  if (idx_closest_right >= 0)
  {
    double width_angle_right = std::atan(half_car_width / this->closest_dist_right);
    double max_right_goal = this->closest_angle_right + width_angle_right; // move goal back toward center if right wall is too close
    this->goal_angle = std::max(this->goal_angle, max_right_goal); // only adjust if goal angle is inside car width area
  }

  if (idx_closest_left >= 0)
  {
    double width_angle_left = std::atan(half_car_width / this->closest_dist_left);
    double min_left_goal = this->closest_angle_left - width_angle_left; // move goal back toward center if left wall is too close
    this->goal_angle = std::min(this->goal_angle, min_left_goal); // only adjust if goal angle is inside car width area
  }

  this->goal_angle_before_steer_clamp = this->goal_angle; // Save for visualization

  const double STEER_LIMIT = 0.2;

  // limit the correction so it does not exceed the steer limit when added to the goal angle
  if (goal_angle >= 0.0)
  {
    double max_additional_right = std::max(0.0, STEER_LIMIT - goal_angle);
    correction_right = std::min(correction_right, max_additional_right);
  }
  if (goal_angle <= 0.0)
  {
    double max_additional_left = std::max(0.0, STEER_LIMIT + goal_angle);
    correction_left = std::min(correction_left, max_additional_left);
  }

  // set final goal angle 
  goal_angle = (goal_angle - correction_left + correction_right);


  //// SPEED ////

  this->get_parameter("max_speed", this->MAXDRIVESPEED);

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

  this->diag_dist_closest_front = min_distance; // Save for diagnostics

  double speed_factor;
  
  speed_factor = 1 - std::exp(-(this->BRAKE_TUNE) * (min_distance-0.01)); // offset exp by 0.1 so min value is above 0

  double set_speed = this->MAXDRIVESPEED * speed_factor;
  this->diag_set_speed = set_speed; // Save for diagnostics

  //// PUBLISH ////

  ackermann_msgs::msg::AckermannDriveStamped drive_msg;

  drive_msg.drive.steering_angle = std::clamp(static_cast<float>(this->goal_angle), -0.34f, 0.34f); // should be 0.34
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
  add_kv("2_target_speed", this->diag_set_speed);
  add_kv("3_active_threshold", this->diag_active_threshold);
  add_kv("4_num_gaps_found", static_cast<double>(this->all_gaps.size()));
  add_kv("5_dist_closest_front", this->diag_dist_closest_front);
  add_kv("6_dist_closest_left", this->diag_dist_closest_left);
  add_kv("7_dist_closest_right", this->diag_dist_closest_right);
  add_kv("8_correction_left", this->diag_correction_left);
  add_kv("9_correction_right", this->diag_correction_right);

  diag_msg.status.push_back(status);
  diag_pub->publish(diag_msg);
}

/*
void FollowGap::disparity_extender(
    const std::vector<float> &ranges,
    int n_extend,
    float step_depth,
    std::vector<float> &out_extended_ranges
)
{
  int num_lasers = ranges.size();
  out_extended_ranges = ranges;

  for (int i = 0; i < num_lasers; i++)
  {
    float current = ranges[i];
    if (!std::isfinite(current)) continue;

    // prev calculation: min of ranges[i-n_extend ... i-1]
    float prev = std::numeric_limits<float>::infinity();
    for (int j = std::max(0, i - n_extend); j < i; j++) {
        if (std::isfinite(ranges[j])) prev = std::min(prev, ranges[j]);
    }

    // next calculation: min of ranges[i+1 ... i+n_extend]
    float next = std::numeric_limits<float>::infinity();
    for (int j = i + 1; j <= std::min(num_lasers - 1, i + n_extend); j++) {
        if (std::isfinite(ranges[j])) next = std::min(next, ranges[j]);
    }

    if (std::isfinite(prev) && (current - prev > step_depth)) 
    {
      out_extended_ranges[i] = std::min(out_extended_ranges[i], prev);
      std::cout << "extended at " << i << " prev: " << prev << " current: " << current << std::endl;
    }
    
    if (std::isfinite(next) && (current - next > step_depth)) 
    {
      out_extended_ranges[i] = std::min(out_extended_ranges[i], next);
      std::cout << "extended at " << i << " next: " << next << " current: " << current << std::endl;
    }
  }
}
*/
void FollowGap::disparity_extender(
    const std::vector<float> &ranges,
    int n_extend,
    float step_depth,
    std::vector<float> &out_extended_ranges
)
{
  int num_lasers = ranges.size();
  out_extended_ranges = ranges; // Initialize with original ranges

  // Iterate through adjacent pairs of rays
  for (int i = 0; i < num_lasers - 1; i++)
  {
    float r1 = ranges[i];
    float r2 = ranges[i + 1];

    // Skip if either ray is invalid
    if (!std::isfinite(r1) || !std::isfinite(r2)) continue;

    // Check if the jump between ADJACENT rays is larger than the threshold
    if (std::abs(r1 - r2) > step_depth)
    {
      if (r1 < r2)
      {
        // r1 is the closer point (obstacle edge). Extend r1 into the r2 (gap) side.
        // Overwrite the next 'n_extend' indices to the right.
        for (int j = i + 1; j <= std::min(num_lasers - 1, i + n_extend); j++)
        {
          out_extended_ranges[j] = std::min(out_extended_ranges[j], r1);
        }
      }
      else
      {
        // r2 is the closer point. Extend r2 into the r1 side.
        // Overwrite the previous 'n_extend' indices to the left.
        for (int j = i; j >= std::max(0, i + 1 - n_extend); j--)
        {
          out_extended_ranges[j] = std::min(out_extended_ranges[j], r2);
        }
      }
    }
  }
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

  const double MAX_RELEVANT_ANGLE = M_PI_2; // ignore walls behind the car
  const double REAR_CLEARANCE = 0.2;      // ignore obstacles too far behind for a rear tire hit

  auto is_relevant_ray = [&](int idx) {
    double angle = idx * angle_increment + angle_min;
    if (std::abs(angle) > MAX_RELEVANT_ANGLE)
      return false;
    double distance = static_cast<double>(ranges[idx]);
    if (!std::isfinite(distance) || distance <= 0.0)
      return false;
    double x = std::cos(angle) * distance;
    return x >= -REAR_CLEARANCE;
  };

  for (int k = static_cast<int>(num_lasers * this->GAP_VIEW_ANGLE_PROCENT); k < static_cast<int>(num_lasers * (1 - this->GAP_VIEW_ANGLE_PROCENT)); k++)
  {
    if (!is_relevant_ray(k))
      continue;

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

  // visualize the closest wall points that are used for correction
  if (this->closest_idx_right >= 0)
  {
    double x = std::cos(this->closest_angle_right) * this->closest_dist_right;
    double y = std::sin(this->closest_angle_right) * this->closest_dist_right;
    marker_array.markers.push_back(this->create_marker(x, y, 1.0, 0.0, 0.0, 2.0, 1001));
    marker_array.markers.push_back(this->create_line_marker(0.0, 0.0, x, y, 1.0, 0.0, 0.0, 0.01, 1002));
  }
  if (this->closest_idx_left >= 0)
  {
    double x = std::cos(this->closest_angle_left) * this->closest_dist_left;
    double y = std::sin(this->closest_angle_left) * this->closest_dist_left;
    marker_array.markers.push_back(this->create_marker(x, y, 0.0, 1.0, 0.0, 2.0, 1003));
    marker_array.markers.push_back(this->create_line_marker(0.0, 0.0, x, y, 0.0, 1.0, 0.0, 0.01, 1004));
  }

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

  // project goal angle before width clamp
  double x_before_width = std::cos(this->goal_angle_before_width_clamp) * 1;
  double y_before_width = std::sin(this->goal_angle_before_width_clamp) * 1;
  marker_array.markers.push_back(this->create_line_marker(0.0, 0.0, x_before_width, y_before_width, 0.0, 1.0, 0.0, 0.02, 5001));
  marker_array.markers.push_back(this->create_marker(x_before_width, y_before_width, 0.0, 1.0, 0.0, 1.5, 5002));

  // project goal angle before steer-limit clamp
  double x_before_steer = std::cos(this->goal_angle_before_steer_clamp) * 1;
  double y_before_steer = std::sin(this->goal_angle_before_steer_clamp) * 1;
  marker_array.markers.push_back(this->create_line_marker(0.0, 0.0, x_before_steer, y_before_steer, 0.0, 0.0, 1.0, 0.02, 5003));
  marker_array.markers.push_back(this->create_marker(x_before_steer, y_before_steer, 0.0, 0.0, 1.0, 1.5, 5004));

  // project final goal angle
  double x = std::cos(this->goal_angle) * 1;
  double y = std::sin(this->goal_angle) * 1;

  visualization_msgs::msg::Marker marker = this->create_marker(x, y, 1.0, 0.0, 0.0, 3.0, num_lasers);
  marker_array.markers.push_back(marker);

  // visualize the gaussian bell used for correction as an ellipse
  visualization_msgs::msg::Marker ellipse_marker;
  ellipse_marker.header.frame_id = "ego_racecar/laser";
  ellipse_marker.header.stamp = this->get_clock()->now();
  ellipse_marker.ns = "gaussian_ellipse";
  ellipse_marker.id = 999; 
  ellipse_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  ellipse_marker.action = visualization_msgs::msg::Marker::ADD;
  
  ellipse_marker.scale.x = 0.02;
  ellipse_marker.color.r = 1.0;
  ellipse_marker.color.g = 0.5;
  ellipse_marker.color.b = 0.0;
  ellipse_marker.color.a = 0.8;

  double sigma_f = 2.0; 
  double sigma_s = 0.3;
  double ellipse_offset = 0.5;

  // generate points for the ellipse
  for (double angle = -M_PI; angle <= M_PI; angle += 0.1)
  {
      // calculate radius for the current angle based on the elliptical Gaussian formula
      double r = (sigma_f * sigma_s) /
          std::sqrt(std::pow(sigma_s * std::cos(angle), 2) + std::pow(sigma_f * std::sin(angle), 2));
      
      geometry_msgs::msg::Point p;
      p.x = r * std::cos(angle) + ellipse_offset;
      p.y = r * std::sin(angle);
      p.z = 0.0;
      
      ellipse_marker.points.push_back(p);
  }
  
  // close the ellipse by connecting the last point to the first
  if (!ellipse_marker.points.empty()) {
      ellipse_marker.points.push_back(ellipse_marker.points.front());
  }

  marker_array.markers.push_back(ellipse_marker);

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