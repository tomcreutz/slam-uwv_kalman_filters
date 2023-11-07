#ifndef UWV_KALMAN_FILTERS_MEASUREMENT_MODELS_LINEAR_VELOCITY_HPP
#define UWV_KALMAN_FILTERS_MEASUREMENT_MODELS_LINEAR_VELOCITY_HPP

#include <Eigen/Geometry>
// todo(tcreutz): is this good like this?
#include "../PoseState.hpp"

namespace uwv_kalman_filters
{

template<typename FilterState>
VelocityType
measurementVelocity(
  const FilterState & state, const Eigen::Affine3d & imu_in_dvl,
  const Eigen::Vector3d & rotation_rate)
{
  // return expected velocity in dvl frame
  auto velocity = VelocityType(
    imu_in_dvl.linear() * state.orientation.conjugate() * state.velocity + imu_in_dvl.translation().cross(
      imu_in_dvl.linear() * (rotation_rate - state.bias_gyro)));
  return velocity;
  //return VelocityType(state.orientation.inverse() * state.velocity);
}

} // namespace uwv_kalman_filters

#endif
