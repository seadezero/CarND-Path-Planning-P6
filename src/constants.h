/**
 * constants.h
 * Constants used in this project.
 *
 * Created on: Apr 21, 2021
 * Author: Ying Li
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

// [m/s] Speed limit of the highway driving
constexpr double kSpeedLimit{22.0}; // [m/s]

// [m/s^2] Acceleration limit of the highway driving
constexpr double kAccelLimit{10.0}; // [m/s^2]

// [m/s^3] Jerk limit of the highway driving
constexpr double kJerkLimit{10.0};  // [m/s^3]

#endif // CONSTANTS_H_