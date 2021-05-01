/**
 * trajectory_generator.h
 * trajectory generator class.
 *
 * Created on: Apr 21, 2021
 * Author: Ying Li
 */

#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include "behavior_planner.h"
#include "types.h"

#include <vector>

class TrajectoryGenerator {
public:
  TrajectoryGenerator(const std::vector<double>& map_waypoints_x,
                      const std::vector<double>& map_waypoints_y,
                      const std::vector<double>& map_waypoints_s, 
                      const std::vector<double>& map_waypoints_dx, 
                      const std::vector<double>& map_waypoints_dy);
  ~TrajectoryGenerator();

  // Generate trajectory path starting from a given state
  void GenerateTrajectory(const VehicleState& start_vehicle_state,
                          double target_speed,
                          unsigned int target_lane_idx,
                          std::vector<double>& ref_pts_x,
                          std::vector<double>& ref_pts_y,
                          std::vector<double>& next_x_vals,
                          std::vector<double>& next_y_vals);
  
private:
  Lanes lanes_;
  
  std::vector<double> map_waypoints_x_;
  std::vector<double> map_waypoints_y_;
  std::vector<double> map_waypoints_s_;
  std::vector<double> map_waypoints_dx_;
  std::vector<double> map_waypoints_dy_;
};

#endif // TRAJECTORY_GENERATOR_H_
