#ifndef UWV_KALMAN_FILTERS_MEASUREMENT_MODELS_ACCELERATION_HPP
#define UWV_KALMAN_FILTERS_MEASUREMENT_MODELS_ACCELERATION_HPP

#include <Eigen/Geometry>
// todo(tcreutz): is this good like this?
#include "../PoseState.hpp"

namespace uwv_kalman_filters
{

template<typename FilterState>
AccelerationType
measurementAcceleration(const FilterState & state)
{
  // returns expected accelerations in the IMU frame
  return AccelerationType(
    state.orientation *
    (Eigen::Vector3d(state.acceleration) - Eigen::Vector3d(state.bias_acc)) -
    Eigen::Vector3d(0., 0., state.gravity(0)));
}

} // namespace uwv_kalman_filters

#endif
