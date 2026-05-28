#include "pure_pursuit/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <unordered_map>

namespace pure_pursuit
{
namespace
{

constexpr const char * REQUIRED_COLUMNS[] = {
  "s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2"};

std::string trim(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::vector<std::string> splitSemicolonLine(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ';')) {
    fields.push_back(trim(field));
  }
  return fields;
}

bool looksLikeHeader(const std::vector<std::string> & fields)
{
  return std::find(fields.begin(), fields.end(), "s_m") != fields.end() &&
         std::find(fields.begin(), fields.end(), "x_m") != fields.end() &&
         std::find(fields.begin(), fields.end(), "y_m") != fields.end();
}

std::unordered_map<std::string, std::size_t> buildColumnIndex(
  const std::vector<std::string> & header_fields)
{
  std::unordered_map<std::string, std::size_t> index;
  for (std::size_t i = 0; i < header_fields.size(); ++i) {
    index[header_fields[i]] = i;
  }

  for (const auto * column : REQUIRED_COLUMNS) {
    if (index.find(column) == index.end()) {
      throw std::runtime_error("Raceline CSV is missing required column: " + std::string(column));
    }
  }

  return index;
}

double parseDoubleField(
  const std::vector<std::string> & fields,
  const std::unordered_map<std::string, std::size_t> & column_index,
  const std::string & column,
  std::size_t line_number)
{
  const auto index_it = column_index.find(column);
  if (index_it == column_index.end() || index_it->second >= fields.size()) {
    throw std::runtime_error(
      "Raceline CSV line " + std::to_string(line_number) +
      " is missing value for column: " + column);
  }

  try {
    std::size_t parsed_chars = 0;
    const double value = std::stod(fields[index_it->second], &parsed_chars);
    if (parsed_chars != fields[index_it->second].size() || !std::isfinite(value)) {
      throw std::invalid_argument("non-finite or trailing characters");
    }
    return value;
  } catch (const std::exception &) {
    throw std::runtime_error(
      "Raceline CSV line " + std::to_string(line_number) +
      " has malformed numeric value in column: " + column);
  }
}

bool isStrictlyIncreasingS(const std::vector<RacelinePoint> & raceline)
{
  for (std::size_t i = 1; i < raceline.size(); ++i) {
    if (raceline[i].s_m <= raceline[i - 1].s_m) {
      return false;
    }
  }
  return true;
}

std::size_t findLookaheadByArcLengthFallback(
  const std::vector<RacelinePoint> & raceline,
  std::size_t nearest_index,
  double lookahead_m)
{
  double accumulated_m = 0.0;
  std::size_t previous_index = nearest_index;

  for (std::size_t step = 1; step <= raceline.size(); ++step) {
    const std::size_t index = (nearest_index + step) % raceline.size();
    accumulated_m += std::hypot(
      raceline[index].x_m - raceline[previous_index].x_m,
      raceline[index].y_m - raceline[previous_index].y_m);

    if (accumulated_m >= lookahead_m) {
      return index;
    }
    previous_index = index;
  }

  return nearest_index;
}

}  // namespace

std::vector<RacelinePoint> parseRacelineCsv(const std::string & csv_path)
{
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    throw std::runtime_error("Raceline CSV does not exist or cannot be opened: " + csv_path);
  }

  std::unordered_map<std::string, std::size_t> column_index;
  bool header_seen = false;
  std::vector<RacelinePoint> raceline;
  std::string line;
  std::size_t line_number = 0;

  while (std::getline(file, line)) {
    ++line_number;
    std::string stripped = trim(line);
    if (stripped.empty()) {
      continue;
    }

    if (stripped[0] == '#') {
      stripped = trim(stripped.substr(1));
      const auto header_fields = splitSemicolonLine(stripped);
      if (looksLikeHeader(header_fields)) {
        column_index = buildColumnIndex(header_fields);
        header_seen = true;
      }
      continue;
    }

    const auto fields = splitSemicolonLine(stripped);
    if (!header_seen) {
      if (looksLikeHeader(fields)) {
        column_index = buildColumnIndex(fields);
        header_seen = true;
        continue;
      }
      if (fields.size() < 7) {
        throw std::runtime_error(
          "Raceline CSV line " + std::to_string(line_number) +
          " has fewer than 7 semicolon-separated columns");
      }
      column_index = buildColumnIndex(
        {"s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2"});
      header_seen = true;
    }

    RacelinePoint point;
    point.s_m = parseDoubleField(fields, column_index, "s_m", line_number);
    point.x_m = parseDoubleField(fields, column_index, "x_m", line_number);
    point.y_m = parseDoubleField(fields, column_index, "y_m", line_number);
    point.psi_rad = parseDoubleField(fields, column_index, "psi_rad", line_number);
    point.kappa_radpm = parseDoubleField(fields, column_index, "kappa_radpm", line_number);
    point.vx_mps = parseDoubleField(fields, column_index, "vx_mps", line_number);
    point.ax_mps2 = parseDoubleField(fields, column_index, "ax_mps2", line_number);
    raceline.push_back(point);
  }

  if (raceline.empty()) {
    throw std::runtime_error("Raceline CSV contains no data rows: " + csv_path);
  }

  if (!isStrictlyIncreasingS(raceline)) {
    throw std::runtime_error("Raceline CSV s_m column must be strictly increasing");
  }

  return raceline;
}

std::size_t findNearestPointIndex(
  const std::vector<RacelinePoint> & raceline,
  double x_m,
  double y_m)
{
  if (raceline.empty()) {
    throw std::invalid_argument("Cannot search nearest point on an empty raceline");
  }

  std::size_t nearest_index = 0;
  double best_distance_sq = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < raceline.size(); ++i) {
    const double dx = raceline[i].x_m - x_m;
    const double dy = raceline[i].y_m - y_m;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      nearest_index = i;
    }
  }

  return nearest_index;
}

double computeClosedTrackLength(const std::vector<RacelinePoint> & raceline)
{
  if (raceline.size() < 2) {
    return 0.0;
  }

  const auto & first = raceline.front();
  const auto & last = raceline.back();
  const double closing_segment_m = std::hypot(last.x_m - first.x_m, last.y_m - first.y_m);
  return std::max(0.0, last.s_m - first.s_m) + closing_segment_m;
}

std::size_t findLookaheadPointIndex(
  const std::vector<RacelinePoint> & raceline,
  std::size_t nearest_index,
  double lookahead_m)
{
  if (raceline.empty()) {
    throw std::invalid_argument("Cannot find lookahead point on an empty raceline");
  }
  if (nearest_index >= raceline.size()) {
    throw std::out_of_range("Nearest raceline index is out of range");
  }
  if (raceline.size() == 1 || lookahead_m <= 0.0) {
    return nearest_index;
  }
  if (!isStrictlyIncreasingS(raceline)) {
    return findLookaheadByArcLengthFallback(raceline, nearest_index, lookahead_m);
  }

  const double track_length_m = computeClosedTrackLength(raceline);
  if (track_length_m <= 0.0) {
    return findLookaheadByArcLengthFallback(raceline, nearest_index, lookahead_m);
  }

  double target_s_m = raceline[nearest_index].s_m + lookahead_m;

  // std::cout << "  " << std::endl;
  // std::cout << "raceline[nearest_index].s_m: " << raceline[nearest_index].s_m << std::endl;
  // std::cout << "lookahead_m: " << lookahead_m << std::endl;
  // std::cout << "target_s_m: " << target_s_m << std::endl;
  
  target_s_m = std::fmod(target_s_m - raceline.front().s_m, track_length_m) + raceline.front().s_m;

  if (target_s_m >= raceline[nearest_index].s_m) {
    for (std::size_t i = nearest_index; i < raceline.size(); ++i) {
      if (raceline[i].s_m >= target_s_m) {
        // std::cout << "raceline[i].s_m: " << raceline[i].s_m << std::endl;
        return i;
      }
    }
  }

  for (std::size_t i = 0; i <= nearest_index && i < raceline.size(); ++i) {
    if (raceline[i].s_m >= target_s_m) {
      // std::cout << "raceline[i].s_m: " << raceline[i].s_m << std::endl;
      return i;
    }
  }

  return raceline.size() - 1;
}

double computeSpeedDependentLookahead(
  double current_speed_mps,
  double min_lookahead_m,
  double max_lookahead_m,
  double lookahead_gain,
  double min_target_speed,
  double max_target_speed)
{
  double unclamped = min_lookahead_m + lookahead_gain * std::max(0.0, current_speed_mps);

  //const double m = (max_target_speed) / (max_target_speed*max_target_speed) ;
  //unclamped = m * current_speed_mps * current_speed_mps;

  std::cout << "current_speed_mps: " << current_speed_mps << std::endl;
  std::cout << "unclamped: " << unclamped << std::endl;
  return std::clamp(unclamped, min_lookahead_m, max_lookahead_m);
}


int computeCurvatureDependentLookahead(
  std::vector<pure_pursuit::RacelinePoint> raceline,
  int nearest_index,
  double min_lookahead_m,
  double max_lookahead_m,
  double lookahead_gain)
{

  int min_idx = pure_pursuit::findLookaheadPointIndex(raceline, nearest_index, min_lookahead_m);
  int max_idx = pure_pursuit::findLookaheadPointIndex(raceline, nearest_index, max_lookahead_m);

  double min_kappa_radpm = 0;
  double max_kappa_radpm = 1.5;

  int raceline_length = raceline.size();
  raceline[nearest_index];
  int i = nearest_index;

  double delta_distance = raceline[1].s_m - raceline[0].s_m;
  double distance = 0;

  while (true) {

    i += 1;
    distance += delta_distance;
    
    if (distance > max_lookahead_m) { break; }
    if (distance < min_lookahead_m) { continue; } 

    // handle wrap around
    if (i >= raceline_length) { i = 0;}

    // iterate forward untill a strong enough curve is found
    if (std::abs(raceline[i].kappa_radpm) > max_kappa_radpm * lookahead_gain) { break; }
  }
  std::cout << "current lookahaed distance: " << distance << " index: " << i << std::endl;
  return i;
}

double computePurePursuitSteering(
  double lookahead_x_base_m,
  double lookahead_y_base_m,
  double wheelbase_m,
  double max_steering_angle_rad)
{
  const double lookahead_distance_sq =
    lookahead_x_base_m * lookahead_x_base_m + lookahead_y_base_m * lookahead_y_base_m;
  if (lookahead_distance_sq <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  const double curvature = 2.0 * lookahead_y_base_m / lookahead_distance_sq;
  return std::clamp(
    std::atan(wheelbase_m * curvature),
    -std::abs(max_steering_angle_rad),
    std::abs(max_steering_angle_rad));
}

double computeCurvatureLimitedSpeed(
  double target_speed_mps,
  double curvature_radpm,
  double curvature_speed_gain,
  double min_speed_mps)
{
  if (curvature_speed_gain <= 0.0) {
    return std::max(min_speed_mps, target_speed_mps);
  }

  const double scale = 1.0 / (1.0 + curvature_speed_gain * std::abs(curvature_radpm));
  return std::max(min_speed_mps, target_speed_mps * scale);
}

}  // namespace pure_pursuit
