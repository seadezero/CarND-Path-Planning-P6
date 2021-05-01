/**
 * types.h
 * Data types used in this project.
 *
 * Created on: Apr 21, 2021
 * Author: Ying Li
 */

#ifndef TYPES_H_
#define TYPES_H_

#include "constants.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

struct VehicleState {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
};

class Lanes {
public:
  double getClosestDValue(double d) const {
    auto itr = std::min_element(d_values_.begin(), d_values_.end(), [d](double a, double b) {
      return std::labs(a - d) < std::labs(b - d);
    });
    return *itr;
  }
  
  std::size_t getClosestDIndex(double d) const {
    auto itr = std::min_element(d_values_.begin(), d_values_.end(), [d](double a, double b) {
      return std::labs(a - d) < std::labs(b - d);
    });
    return std::distance(d_values_.begin(), itr);
  }
  
  double getDValueFromIndex(std::size_t idx) const {
    return d_values_[idx];
  }

private:
  const std::array<double, 3U> d_values_{{2.0, 6.0, 10.0}};
};

#endif // TYPES_H_
