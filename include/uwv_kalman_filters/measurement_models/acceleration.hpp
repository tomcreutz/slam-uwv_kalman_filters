#ifndef UWV_KALMAN_FILTERS_MEASUREMENT_MODELS_ACCELERATION_HPP
#define UWV_KALMAN_FILTERS_MEASUREMENT_MODELS_ACCELERATION_HPP

#include <Eigen/Geometry>
// todo(tcreutz): is this good like this?
#include "../PoseState.hpp"

namespace uwv_kalman_filters
{

template<typename FilterState>
AccelerationType
measurementAcceleration(const FilterState & state, const Eigen::Vector3d & gravity)
{
  // returns expected accelerations in the IMU frame
  return AccelerationType(
    state.orientation.inverse() *
    (Eigen::Vector3d(state.acceleration) + gravity + Eigen::Vector3d(state.bias_acc)));
}

} // namespace uwv_kalman_filters

#endif
