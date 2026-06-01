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

  std::vector<RacelinePoint> parseRacelineCsv(const std::string &csv_path);

  std::size_t findNearestPointIndex(
      const std::vector<RacelinePoint> &raceline,
      double x_m,
      double y_m,
      int delay_correction);

  std::size_t findLookaheadPointIndex(
      const std::vector<RacelinePoint> &raceline,
      std::size_t nearest_index,
      double lookahead_m);

  double computeClosedTrackLength(const std::vector<RacelinePoint> &raceline);

  double computeMpsDependentLookahead(
      double min_mps,
      double max_mps,
      double current_speed_mps,
      double min_lookahead_m,
      double max_lookahead_m);

  double computeMpsDependentSpeed(
      double mps_min,
      double mps_max,
      double current_speed_mps,
      double min_speed,
      double max_speed);

  double computePurePursuitSteering(
      double lookahead_x_base_m,
      double lookahead_y_base_m,
      double wheelbase_m,
      double max_steering_angle_rad);

}

#endif
