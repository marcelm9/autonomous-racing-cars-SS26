#include "./../include/reactive/reactive.hpp"

FollowGap::FollowGap() : Node("reactive")
{
  this->declare_parameter<std::string>("scan_topic");
  this->declare_parameter<std::string>("odom_topic");
  this->declare_parameter<std::string>("drive_topic");
  /* ---- subscribers ---- */
  scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(this->get_parameter("scan_topic").as_string().c_str(), 10, std::bind(&FollowGap::scan_callback, this, std::placeholders::_1));
  path_sub = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&FollowGap::path_callback, this, std::placeholders::_1));
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(this->get_parameter("odom_topic").as_string().c_str(), 10, std::bind(&FollowGap::odom_callback, this, std::placeholders::_1));

  /* ---- publishers ---- */
  drive_speed_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->get_parameter("drive_topic").as_string().c_str(), 10);
  gap_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/gap_vis", 1);
  diag_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
  oh_no_im_stuck_publisher = this->create_publisher<std_msgs::msg::String>("/oh_no_im_stuck", 1);

  // publish diagnostics once per second
  diag_timer = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&FollowGap::diag_timer_callback, this));

  // variables
  this->declare_parameter<double>("max_speed", this->MAXDRIVESPEED);
  this->declare_parameter<double>("min_speed", this->MINDRIVESPEED);
  this->declare_parameter<double>("distance_threshold_for_max_speed", this->MAXDISTANCE);
  this->declare_parameter<int>("reac_n_extend", this->N_EXTEND);
  this->declare_parameter<double>("reac_step_depth", this->STEP_DEPTH);
  this->declare_parameter<double>("continuity_weight", this->CONTINUITY_WEIGHT);

  // Initialize from parameters
  this->get_parameter("max_speed", this->MAXDRIVESPEED);
  this->get_parameter("min_speed", this->MINDRIVESPEED);
  this->get_parameter("distance_threshold_for_max_speed", this->MAXDISTANCE);
  this->get_parameter("reac_n_extend", this->N_EXTEND);
  this->get_parameter("reac_step_depth", this->STEP_DEPTH);
  this->get_parameter("continuity_weight", this->CONTINUITY_WEIGHT);

  // Register the dynamic parameter callback
  callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&FollowGap::parameters_callback, this, std::placeholders::_1));
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

rcl_interfaces::msg::SetParametersResult FollowGap::parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters)
    {
        if (param.get_name() == "max_speed") {
            this->MAXDRIVESPEED = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter 'max_speed' updated to: %.2f", this->MAXDRIVESPEED);
        }
        else if (param.get_name() == "min_speed") { // Handle dynamic updates for min_speed
            this->MINDRIVESPEED = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter 'min_speed' updated to: %.2f", this->MINDRIVESPEED);
        }
        else if (param.get_name() == "distance_threshold_for_max_speed") {
            this->MAXDISTANCE = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter 'distance_threshold_for_max_speed' updated to: %.2f", this->MAXDISTANCE);
        }
        else if (param.get_name() == "continuity_weight") {
            this->CONTINUITY_WEIGHT = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter 'continuity_weight' updated to: %.2f", this->CONTINUITY_WEIGHT);
        }
        else if (param.get_name() == "reac_n_extend") {
            this->N_EXTEND = param.as_int();
        }
        else if (param.get_name() == "reac_step_depth") {
            this->STEP_DEPTH = param.as_double();
        }
    }
    return result;
}

void FollowGap::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
  this->scan_msg = *scan_msg;

  double angle_increment = scan_msg->angle_increment;
  double angle_min = scan_msg->angle_min;

  std::vector<float> raw_ranges = scan_msg->ranges;
  
  // 1. Preprocess: Handle NaN/Inf and apply Median Filter
  std::vector<float> ranges_cleaned;
  for (float r : raw_ranges) {
    ranges_cleaned.push_back(std::isfinite(r) ? r : scan_msg->range_max);
  }
  std::vector<float> ranges = this->preprocess_scan(ranges_cleaned);

  //// DISPARITY EXTENSION ////

  std::vector<float> extended_ranges;
  std::vector<int> extended_ranges_mask; // 1 if the range was extended to right , 2 if extended to left, 0 if original
  this->disparity_extender(ranges, N_EXTEND, STEP_DEPTH, extended_ranges, extended_ranges_mask);

  //// DYNAMIC GAP SEARCH ////
  // look for a gap far away and if none is found reduce the distance until a gap is found

  for (int i = 0; i < this->DYN_THRESHOLD_INCREMENT; i++)
  {
    double threshold = this->MAX_THRESHOLD - (this->MAX_THRESHOLD - this->MIN_THRESHOLD) * i / this->DYN_THRESHOLD_INCREMENT;
    this->find_gaps_by_threshold(ranges, threshold, angle_min, angle_increment, all_gaps, gap_angles);
    if (!gap_angles.empty())
    {
      this->diag_active_threshold = threshold; // Save for diagnostics
      break;
    }
  }

  this->all_gaps = all_gaps;
  this->gap_angles = gap_angles;

  //// STEERING WITH WALL AVOIDANCE////
  // Steer to best gap using continuity scoring

  double chosen_gap_dist = 0.0; // Track the distance of the selected gap for dynamic speed

  if (!all_gaps.empty())
  {
    double best_score = -std::numeric_limits<double>::infinity();
    double best_angle = 0.0;

    for (const auto& gap : all_gaps) {
      if (gap.empty()) continue;

      // 1. Calculate representative depth (average of top X%)
      std::vector<double> gap_dists;
      for (double angle : gap) {
        int idx = static_cast<int>((angle - angle_min) / angle_increment);
        gap_dists.push_back(extended_ranges[idx]);
      }
      std::sort(gap_dists.begin(), gap_dists.end(), std::greater<double>());
      
      int num_top = std::max(1, static_cast<int>(gap_dists.size() * this->TOP_PERCENTAGE));
      double avg_dist = 0;
      for (int j = 0; j < num_top; ++j) avg_dist += gap_dists[j];
      avg_dist /= num_top;

      // 2. Calculate center angle of the gap
      double gap_angle = gap[gap.size() / 2];

      // 3. Continuity Scoring
      double angle_diff = std::abs(gap_angle - this->last_goal_angle);
      double score = avg_dist - (angle_diff * this->CONTINUITY_WEIGHT);

      if (score > best_score) {
        best_score = score;
        best_angle = gap_angle;
        chosen_gap_dist = avg_dist;
      }
    }

    this->goal_angle = best_angle;

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
  int num_lasers = ranges.size();
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
  }

  if (idx_closest_left >= 0) {
    this->closest_idx_left = idx_closest_left;
    this->closest_angle_left = idx_closest_left * angle_increment + angle_min;
    this->closest_dist_left = ranges[idx_closest_left];
    this->diag_dist_closest_left = this->closest_dist_left;
  } else {
    this->closest_idx_left = -1;
  }

  this->diag_correction_left = correction_left;
  this->diag_correction_right = correction_right;

  this->goal_angle_before_width_clamp = this->goal_angle; // Save for visualization

  // Geometric safety: ensure we steer far enough to clear the car's width
  const double half_car_width = 0*6*0.2032 / 2.0; // approx 0.4m

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

  // if goal angle is within an area which is extended by the disparity extender, push angle to edge of extention area
  int goal_angle_ray_index = static_cast<int>((this->goal_angle - angle_min) / angle_increment);

  // check if goal angle is within extended area
  if(goal_angle_ray_index >= 0 && goal_angle_ray_index < extended_ranges.size() && extended_ranges_mask[goal_angle_ray_index] >> 0)
  {
    // push the goal angle away from that wall by at least the number of rays it was extended
    if (extended_ranges_mask[goal_angle_ray_index] == 1) // 1 -> extended to right side
    {
      // right side, push right until mask is 0
      // find how many rays to go right
       int new_goal_index = goal_angle_ray_index;
       while (new_goal_index < extended_ranges_mask.size() && extended_ranges_mask[new_goal_index] == 1)
       {
         new_goal_index++;
       }
       this->goal_angle = new_goal_index * angle_increment + angle_min; 
    }
    else // mask is 2 -> extended to left side
    {
      // left side, push left until mask is 0
      // find how many rays to go left
       int new_goal_index = goal_angle_ray_index;
       while (new_goal_index >= 0 && extended_ranges_mask[new_goal_index] == 2)
       {
         new_goal_index--;
       }
       this->goal_angle = new_goal_index * angle_increment + angle_min;
    }
  }


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
  this->get_parameter("min_speed", this->MINDRIVESPEED);

  // Keep front-distance check purely for diagnostic visualization
  float min_front_distance = std::numeric_limits<float>::infinity();
  for (int i = num_lasers / 2 - 10; i < num_lasers / 2 + 10; i++)
  {
    if (ranges[i] < min_front_distance)
    {
      min_front_distance = ranges[i];
    }
  }
  this->diag_dist_closest_front = min_front_distance; // Save for diagnostics

  // Angle Factor: Slow down as steering angle increases
  double max_steering_angle = 0.41; // Using your clamp limit
  double angle_penalty = std::min(std::abs(this->goal_angle) / max_steering_angle, 1.0);
  double angle_factor = 1.0 - angle_penalty; // 1.0 = straight, 0.0 = full lock

  // Distance Factor: Normalize chosen gap depth (0.0 = close, 1.0 = max depth)
  double distance_factor = 0.0;
  if (this->MAX_THRESHOLD > 0.0) {
    distance_factor = std::min(chosen_gap_dist / this->MAXDISTANCE, 1.0);
  }

  // Combine both factors to determine optimal speed multiplier [0.0, 1.0]
  double speed_factor = distance_factor * angle_factor;

  // Force speed multiplier to 0 if the robot is stuck
  if (oh_no_im_stuck) {
    speed_factor = 0.0;
  }

  // Scale the multiplier into the settable Minimum and Maximum speeds
  double set_speed = this->MINDRIVESPEED + (speed_factor * (this->MAXDRIVESPEED - this->MINDRIVESPEED));
  this->diag_set_speed = set_speed; // Save for diagnostics

  //// PUBLISH ////

  ackermann_msgs::msg::AckermannDriveStamped drive_msg;

  drive_msg.drive.steering_angle = std::clamp(static_cast<float>(this->goal_angle), -0.41f, 0.41f); // should be 0.34
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

std::vector<float> FollowGap::preprocess_scan(const std::vector<float> &ranges)
{
    if (this->FILTER_WINDOW_SIZE <= 1) return ranges;

    std::vector<float> smoothed = ranges;
    int half = this->FILTER_WINDOW_SIZE / 2;
    int n = ranges.size();

    for (int i = 0; i < n; ++i) {
        std::vector<float> window;
        for (int j = std::max(0, i - half); j <= std::min(n - 1, i + half); ++j) {
            window.push_back(ranges[j]);
        }
        std::sort(window.begin(), window.end());
        smoothed[i] = window[window.size() / 2];
    }
    return smoothed;
}

void FollowGap::disparity_extender(
    const std::vector<float> &ranges,
    int n_extend,
    float step_depth,
    std::vector<float> &out_extended_ranges,
    std::vector<int> &out_extended_ranges_mask
)
{
  int num_lasers = ranges.size();
  out_extended_ranges = ranges; // Initialize with original ranges
  out_extended_ranges_mask = std::vector<int>(num_lasers, 0); // Initialize mask 

  // Iterate through adjacent pairs of rays
  for (int i = 0; i < num_lasers - 1; i++)
  {
    float r1 = ranges[i];
    float r2 = ranges[i + 1];

    // Skip if either ray is invalid
    if (!std::isfinite(r1) || !std::isfinite(r2)) continue;

    // Check if the jump between adjacent rays is larger than the threshold
    if (std::abs(r1 - r2) > step_depth)
    {
      if (r1 < r2)
      {
        // r1 is the closer point (obstacle edge). Extend r1 into the r2 (gap) side.
        // Linear scaling: Closer obstacles get larger extensions.
        // At 0m -> 100% of n_extend. At 10m -> 10% of n_extend.
        double scale = std::max(0.1, 1.0 - (static_cast<double>(r1) / 5.0));
        int current_n = static_cast<int>(std::floor(n_extend * scale));
        current_n = std::max(1, current_n); // Ensure at least 1 ray is extended
        
        for (int j = i + 1; j <= std::min(num_lasers - 1, i + current_n); j++)
        {
          if (r1 < out_extended_ranges[j]) {
            out_extended_ranges[j] = r1;
            out_extended_ranges_mask[j] = 1; // extended to the right
          }
        }
      }
      else
      {
        // r2 is the closer point. Extend r2 into the r1 side.
        double scale = std::max(0.1, 1.0 - (static_cast<double>(r2) / 10.0));
        int current_n = static_cast<int>(std::floor(n_extend * scale));
        current_n = std::max(1, current_n);
        
        // Overwrite the previous 'current_n' indices to the left.
        for (int j = i; j >= std::max(0, i + 1 - current_n); j--)
        {
          if (r2 < out_extended_ranges[j]) {
            out_extended_ranges[j] = r2;
            out_extended_ranges_mask[j] = 2; // extended to the left
          }
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
  double longest_ray_in_gap = 0.0;
  double longest_gap = 0.0;

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
      longest_ray_in_gap = std::max(longest_ray_in_gap, static_cast<double>(ranges[k]));
    }
    else
    {
      // end of Gap

      if (min_count > this->N_GAP_HITS && longest_ray_in_gap > longest_gap)
      {
        // new best gap
        out_best_gap = temp_angles;
        longest_gap = longest_ray_in_gap;
      }

      // safe as a found gap
      if (!temp_angles.empty() && min_count > this->N_GAP_HITS)
        out_all_gaps.push_back(temp_angles);

      // reset temporary values for next gap
      temp_angles.clear();
      min_count = 0;
      longest_ray_in_gap = 0.0;
    }
  }

  // edge case: scan ends with a gap still "open"
  if (!temp_angles.empty() && min_count > this->N_GAP_HITS)
  {
    out_all_gaps.push_back(temp_angles);
    if (longest_ray_in_gap > longest_gap)
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
  ellipse_marker.header.frame_id = this->scan_msg.header.frame_id;
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

  marker.header.frame_id = this->scan_msg.header.frame_id;
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

  marker.header.frame_id = this->scan_msg.header.frame_id;
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