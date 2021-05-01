/**
 * trajectory_generator.cpp
 * trajectory generator class.
 *
 * Created on: Apr 21, 2021
 * Author: Ying Li
 */

#include "trajectory_generator.h"

#include "constants.h"
#include "helpers.h"
#include "spline.h"

#include <algorithm>
#include <iostream>

TrajectoryGenerator::TrajectoryGenerator(const std::vector<double>& map_waypoints_x,
                                         const std::vector<double>& map_waypoints_y,
                                         const std::vector<double>& map_waypoints_s, 
                                         const std::vector<double>& map_waypoints_dx, 
                                         const std::vector<double>& map_waypoints_dy)
  : map_waypoints_x_{map_waypoints_x}
  , map_waypoints_y_{map_waypoints_y}
  , map_waypoints_s_{map_waypoints_s}
  , map_waypoints_dx_{map_waypoints_dx}
  , map_waypoints_dy_{map_waypoints_dy}{}

TrajectoryGenerator::~TrajectoryGenerator() {}

void TrajectoryGenerator::GenerateTrajectory(const VehicleState& start_vehicle_state,
                                             double target_speed,
                                             unsigned int target_lane_idx,
                                             std::vector<double>& ref_pts_x,
                                             std::vector<double>& ref_pts_y,
                                             std::vector<double>& next_x_vals,
                                             std::vector<double>& next_y_vals) { 
  // Add the rest reference points from new path
  double ref_d = lanes_.getDValueFromIndex(target_lane_idx);
  auto ref_pt1 = getXY(start_vehicle_state.s + 30.0, ref_d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  auto ref_pt2 = getXY(start_vehicle_state.s + 60.0, ref_d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  auto ref_pt3 = getXY(start_vehicle_state.s + 90.0, ref_d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  
  ref_pts_x.push_back(ref_pt1[0]);
  ref_pts_x.push_back(ref_pt2[0]);
  ref_pts_x.push_back(ref_pt3[0]);
  ref_pts_y.push_back(ref_pt1[1]);
  ref_pts_y.push_back(ref_pt2[1]);
  ref_pts_y.push_back(ref_pt3[1]);
  
  // Transform reference points of the spline to vehicle coordinate frame
  for (auto i = 0U; i < ref_pts_x.size(); ++i) {
    double shift_x = ref_pts_x[i] - start_vehicle_state.x;
    double shift_y = ref_pts_y[i] - start_vehicle_state.y;
    
    ref_pts_x[i] = shift_x * cos(0 - start_vehicle_state.yaw) - shift_y * sin(0 - start_vehicle_state.yaw);
    ref_pts_y[i] = shift_x * sin(0 - start_vehicle_state.yaw) + shift_y * cos(0 - start_vehicle_state.yaw);
  }
  
  // Create spline by reference x points and y points
  tk::spline spl;
  spl.set_points(ref_pts_x, ref_pts_y);
  
  // Use the spline to interpolate vehicle trajectory to be generated
  double target_x_vcs = 30.0;
  double target_y_vcs = spl(target_x_vcs);
  double target_dist = distance(target_x_vcs, target_y_vcs, 0.0, 0.0);
  
  double x_add_on = 0.0;
  double ref_speed = start_vehicle_state.speed;
  auto new_path_size = 50U - next_x_vals.size();
  for (auto i = 0U; i <= new_path_size; ++i) {
    double N = target_dist / (0.02 * ref_speed);
    double x_vcs = x_add_on + target_x_vcs/N;
    double y_vcs = spl(x_vcs);
    
    x_add_on = x_vcs;
    
    double x_wcs = x_vcs * cos(start_vehicle_state.yaw) - y_vcs * sin(start_vehicle_state.yaw) + start_vehicle_state.x;
    double y_wcs = x_vcs * sin(start_vehicle_state.yaw) + y_vcs * cos(start_vehicle_state.yaw) + start_vehicle_state.y;
    
    next_x_vals.push_back(x_wcs);
    next_y_vals.push_back(y_wcs);
    if (ref_speed + 0.1 < target_speed) {
      ref_speed += 0.1;  // 0.1 * 50 = 5, so 5m/s^2
    } else if (ref_speed > target_speed) {
      ref_speed -= 0.1;  // 0.1 * 50 = 5, so 5m/s^2
    }
  }
}
