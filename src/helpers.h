/**
 * helpers.h
 * helper functions used in this project.
 *
 * Created on: Apr 21, 2021
 * Author: Ying Li
 */

#ifndef HELPERS_H
#define HELPERS_H

#include "types.h"

#include <math.h>
#include <string>
#include <vector>

std::string hasData(std::string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, 
                    const std::vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, 
                 const std::vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, 
                              const std::vector<double> &maps_x, 
                              const std::vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, 
                          const std::vector<double> &maps_x, 
                          const std::vector<double> &maps_y);

// Copy the left old path points to the beginning of the new path
void CopyPath(const std::vector<double>& previous_path, std::vector<double>& new_path);

// Get start vehicle state and start reference points for spline based on previous path info
void getStartVehicleStateAndReferencePoints(const VehicleState& vehicle_state,
                                            double end_path_s,
                                            double end_path_d,
                                            const std::vector<double>& previous_path_x,
                                            const std::vector<double>& previous_path_y,
                                            VehicleState& start_vehicle_state,
                                            std::vector<double>& ref_pts_x,
                                            std::vector<double>& ref_pts_y);

#endif // HELPERS_H