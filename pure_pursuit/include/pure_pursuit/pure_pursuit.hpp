#ifndef PURE_PURSUIT
#define PURE_PURSUIT

#include <cstddef>
#include <string>
#include <vector>

namespace pure_pursuit
{

struct RacelinePoint
{
  double s_m = 0.0;
  double x_m = 0.0;
  double y_m = 0.0;
  double psi_rad = 0.0;
  double kappa_radpm = 0.0;
  double vx_mps = 0.0;
  double ax_mps2 = 0.0;
};

std::vector<RacelinePoint> parseRacelineCsv(const std::string & csv_path);

std::size_t findNearestPointIndex(
  const std::vector<RacelinePoint> & raceline,
  double x_m,
  double y_m);

std::size_t findLookaheadPointIndex(
  const std::vector<RacelinePoint> & raceline,
  std::size_t nearest_index,
  double lookahead_m);

double computeClosedTrackLength(const std::vector<RacelinePoint> & raceline);

double computeSpeedDependentLookahead(
  double current_speed_mps,
  double min_lookahead_m,
  double max_lookahead_m,
  double lookahead_gain,
  double min_target_speed,
  double max_target_speed);

int computeCurvatureDependentLookahead(
  std::vector<pure_pursuit::RacelinePoint> raceline,
  int nearest_index,
  double min_lookahead_m,
  double max_lookahead_m,
  double lookahead_gain);

double computePurePursuitSteering(
  double lookahead_x_base_m,
  double lookahead_y_base_m,
  double wheelbase_m,
  double max_steering_angle_rad);

double computeCurvatureLimitedSpeed(
  double target_speed_mps,
  double curvature_radpm,
  double curvature_speed_gain,
  double min_speed_mps);





}  // namespace pure_pursuit

#endif  // PURE_PURSUIT
