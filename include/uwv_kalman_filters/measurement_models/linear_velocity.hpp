#ifndef UWV_KALMAN_FILTERS_MEASUREMENT_MODELS_LINEAR_VELOCITY_HPP
#define UWV_KALMAN_FILTERS_MEASUREMENT_MODELS_LINEAR_VELOCITY_HPP

#include <Eigen/Geometry>
// todo(tcreutz): is this good like this?
#include "../PoseState.hpp"

namespace uwv_kalman_filters
{

template<typename FilterState>
VelocityType
measurementVelocity(const FilterState & state)
{
  // return expected velocities in the IMU frame
  return VelocityType(state.orientation.inverse() * state.velocity);
}

} // namespace uwv_kalman_filters

#endif
