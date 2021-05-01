/**
 * behavior_planner.cpp
 * behavior planner class.
 *
 * Created on: Apr 21, 2021
 * Author: Ying Li
 */

#include "behavior_planner.h"

#include <iostream>
#include <limits>

BehaviorPlanner::BehaviorPlanner() {}

BehaviorPlanner::~BehaviorPlanner() {}

void BehaviorPlanner::PlanBehavior(const VehicleState& start_vehicle_state,
                                   const std::vector<std::vector<double>>& sensor_fusion,
                                   std::size_t forward_steps) {

  switch (current_state_) {
    case Maneuver::KL:
      target_speed_ = kSpeedLimit;
      
      // Check if there's a slow vehicle in the near front of the ego car
      if (WillCollideFront(start_vehicle_state, sensor_fusion, forward_steps)) {
        current_state_ = ChoosePrepareLaneChange(start_vehicle_state, sensor_fusion, forward_steps);
      }
      break;
      
    case Maneuver::PLCL:
      
      // Adjust the speed of the ego car based on the speed of the slow vehicle in the front
      AdjustTargetSpeed(start_vehicle_state, sensor_fusion, forward_steps);
      
      // Check if it is ready to start changing lane
      if (isReadyToChangeLane(start_vehicle_state, sensor_fusion, forward_steps)) {
        current_state_ = Maneuver::LCL;
      } else {
        current_state_ = ChoosePrepareLaneChange(start_vehicle_state, sensor_fusion, forward_steps);
      }
      break;

    case Maneuver::PLCR:
      
      // Adjust the speed of the ego car based on the speed of the slow vehicle in the front
      AdjustTargetSpeed(start_vehicle_state, sensor_fusion, forward_steps);
      
      // Check if it is ready to start changing lane
      if (isReadyToChangeLane(start_vehicle_state, sensor_fusion, forward_steps)) {
        current_state_ = Maneuver::LCR;
      } else {
        current_state_ = ChoosePrepareLaneChange(start_vehicle_state, sensor_fusion, forward_steps);
      }
      break;
      
    case Maneuver::LCL:
      
      // Check if the lane changing manuevor has finished
      if (ReachedTargetLane(start_vehicle_state)) {
        current_state_ = Maneuver::KL;
      }
      break;
      
    case Maneuver::LCR:
      
      // Check if the lane changing manuevor has finished
      if (ReachedTargetLane(start_vehicle_state)) {
        current_state_ = Maneuver::KL;
      }
      break;
  }
}

bool BehaviorPlanner::WillCollideFront(const VehicleState& start_vehicle_state,
                                       const std::vector<std::vector<double>>& sensor_fusion,
                                       std::size_t forward_steps) {
  bool will_collide{false};
  target_speed_ = kSpeedLimit;
  auto ego_d_index = lanes_.getClosestDIndex(start_vehicle_state.d);
  for (const auto& vehicle : sensor_fusion) {
    if (lanes_.getClosestDIndex(vehicle[6]) == ego_d_index) {
      double vx = vehicle[3];
      double vy = vehicle[4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_s = vehicle[5] + forward_steps * check_speed * 0.02;
      
      if (check_s > start_vehicle_state.s && check_s - start_vehicle_state.s < 30.0) {
        
        // change target speed to below the front vehicle's speed, to avoid collision
        target_speed_ = std::min(target_speed_, check_speed - 2);
        will_collide = true;
        
        if (check_s - start_vehicle_state.s < 10.0) {
          target_speed_ = std::max(0.0, check_s - 15.0);
        }
      }
    }
  }
  return will_collide;
}

void BehaviorPlanner::AdjustTargetSpeed(const VehicleState& start_vehicle_state,
                                        const std::vector<std::vector<double>>& sensor_fusion,
                                        std::size_t forward_steps) {
  auto ego_d_index = lanes_.getClosestDIndex(start_vehicle_state.d);
  target_speed_ = kSpeedLimit;
  for (const auto& vehicle : sensor_fusion) {
    if (lanes_.getClosestDIndex(vehicle[6]) == ego_d_index) {
      double vx = vehicle[3];
      double vy = vehicle[4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_s = vehicle[5] + forward_steps * check_speed * 0.02;
      
      if (check_s > start_vehicle_state.s && check_s - start_vehicle_state.s < 30.0) {
        // change target speed to below the front vehicle's speed, to avoid collision
        target_speed_ = std::min(target_speed_, check_speed - 2);
      }
    }
  }
}

Maneuver BehaviorPlanner::ChoosePrepareLaneChange(const VehicleState& start_vehicle_state,
                                                  const std::vector<std::vector<double>>& sensor_fusion,
                                                  std::size_t forward_steps) {
  auto d_index = lanes_.getClosestDIndex(start_vehicle_state.d);
  if (d_index == 0U) {
    change_lane_idx_ = 1U;
    return Maneuver::PLCR;
  }
  
  if (d_index == 2U) {
    change_lane_idx_ = 1U;
    return Maneuver::PLCL;
  }
  
  // When current lane is middle lane, compare left lane vehicle space with right lane
  // vehicle space.
  double left_lane_behind_s_max = std::numeric_limits<double>::min();
  double left_lane_front_s_min = std::numeric_limits<double>::max();
  double right_lane_behind_s_max = std::numeric_limits<double>::min();
  double right_lane_front_s_min = std::numeric_limits<double>::max();
  
  for (const auto& vehicle : sensor_fusion) {
    auto vehicle_lane_idx = lanes_.getClosestDIndex(vehicle[6]);
    if (vehicle_lane_idx == 0U) {
      double vx = vehicle[3];
      double vy = vehicle[4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_s = vehicle[5] + forward_steps * check_speed * 0.02;
      
      if (check_s > start_vehicle_state.s && check_s < left_lane_front_s_min) {
        left_lane_front_s_min = check_s;
      }
      
      if (check_s < start_vehicle_state.s && check_s > left_lane_behind_s_max) {
        left_lane_behind_s_max = check_s;
      }
    }
    
    if (vehicle_lane_idx == 2U) {
      double vx = vehicle[3];
      double vy = vehicle[4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_s = vehicle[5] + forward_steps * check_speed * 0.02;
      
      if (check_s > start_vehicle_state.s && check_s < right_lane_front_s_min) {
        right_lane_front_s_min = check_s;
      }
      
      if (check_s < start_vehicle_state.s && check_s > right_lane_behind_s_max) {
        right_lane_behind_s_max = check_s;
      }
    }
  }
  
  if ((start_vehicle_state.s - left_lane_behind_s_max < 10) == (start_vehicle_state.s - right_lane_behind_s_max < 10)) {
    if (left_lane_front_s_min < right_lane_front_s_min) {
      change_lane_idx_ = 2U;
      return Maneuver::PLCR;
    } else {
      change_lane_idx_ = 0U;
      return Maneuver::PLCL;
    }
  } else {
    if (start_vehicle_state.s - left_lane_behind_s_max < 10) {
      change_lane_idx_ = 2U;
      return Maneuver::PLCR;
    } else {
      change_lane_idx_ = 0U;
      return Maneuver::PLCL;
    }
  }
}

bool BehaviorPlanner::isReadyToChangeLane(const VehicleState& start_vehicle_state,
                                          const std::vector<std::vector<double>>& sensor_fusion,
                                          std::size_t forward_steps) {
  for (const auto& vehicle : sensor_fusion) {
    if (lanes_.getClosestDIndex(vehicle[6]) == change_lane_idx_) {
      double vx = vehicle[3];
      double vy = vehicle[4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_s = vehicle[5] + forward_steps * check_speed * 0.02;
      
      if (check_s > start_vehicle_state.s) {
        double distance = check_s - start_vehicle_state.s;
        if (distance < 30.0) {
          return false;
        }
        if (distance < 50.0 && check_speed < start_vehicle_state.speed) {
          return false;
        }
      }
      
      if (check_s <= start_vehicle_state.s) {
        if (start_vehicle_state.s - check_s < 30) {
          return false;
        }
        
        if (start_vehicle_state.s - check_s < 50 && check_speed > start_vehicle_state.speed + 5.0) {
          return false;
        }
      }
    }
  }
  target_lane_idx_ = change_lane_idx_;
  return true;
}

bool BehaviorPlanner::ReachedTargetLane(const VehicleState& start_vehicle_state) {
  return std::labs(start_vehicle_state.d - lanes_.getDValueFromIndex(target_lane_idx_)) < 0.4;
}
