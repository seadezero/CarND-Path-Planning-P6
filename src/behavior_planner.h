/**
 * behavior_planner.h
 * behavior planner class.
 *
 * Created on: Apr 21, 2021
 * Author: Ying Li
 */

#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include "constants.h"
#include "types.h"

#include <vector>

enum class Maneuver {
  KL = 0,  // keep lane
  PLCL,    // prepare lane change to left
  PLCR,    // prepare lane change to right
  LCL,     // lane change to left
  LCR,     // lane change to right
};

class BehaviorPlanner {
public:
  BehaviorPlanner();
  ~BehaviorPlanner();
  
  // Plan behavior for the ego vehicle.
  void PlanBehavior(const VehicleState& start_vehicle_state,
                    const std::vector<std::vector<double>>& sensor_fusion,
                    std::size_t forward_steps);
  
  // Get the target speed of this behavior.
  double getTargetSpeed() const noexcept { return target_speed_; };
  
  // Get the target lane of this behavior.
  unsigned int getTargetLaneIdx() const noexcept { return target_lane_idx_; };
  
private:
  bool WillCollideFront(const VehicleState& start_vehicle_state,
                        const std::vector<std::vector<double>>& sensor_fusion,
                        std::size_t forward_steps);
  
  void AdjustTargetSpeed(const VehicleState& start_vehicle_state,
                         const std::vector<std::vector<double>>& sensor_fusion,
                         std::size_t forward_steps);
  
  Maneuver ChoosePrepareLaneChange(const VehicleState& start_vehicle_state,
                                   const std::vector<std::vector<double>>& sensor_fusion,
                                   std::size_t forward_steps);
  
  bool isReadyToChangeLane(const VehicleState& start_vehicle_state,
                           const std::vector<std::vector<double>>& sensor_fusion,
                           std::size_t forward_steps);
  
  bool ReachedTargetLane(const VehicleState& start_vehicle_state);
  
  Lanes lanes_;                          // lane utility class  
  Maneuver current_state_{Maneuver::KL}; // current maneuvor performed by the ego vehicle
  double target_speed_{kSpeedLimit};     // [m/s] target speed of the current maneuvor
  unsigned int target_lane_idx_{1U};     // target lane index of the current maneuvor
  unsigned int change_lane_idx_{1U};     // target lane index of the current maneuvor
};

#endif // BEHAVIOR_PLANNER_H_