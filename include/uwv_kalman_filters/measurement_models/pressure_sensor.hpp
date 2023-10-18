#ifndef UWV_KALMAN_FILTERS_MEASUREMENT_MODELS_PRESSURE_SENSOR_HPP
#define UWV_KALMAN_FILTERS_MEASUREMENT_MODELS_PRESSURE_SENSOR_HPP

#include <Eigen/Geometry>

namespace uwv_kalman_filters
{
template<typename FilterState>
Eigen::Matrix<TranslationType::scalar, 1, 1>
measurementPressureSensor(
  const FilterState & state, const Eigen::Vector3d & pressure_sensor_in_imu,
  const double & atmospheric_pressure, const double & water_density,
  const double & gravity)
{
  Eigen::Vector3d pressure_sensor_in_nav = state.position + state.orientation *
    pressure_sensor_in_imu;
  Eigen::Matrix<TranslationType::scalar, 1, 1> pressure;
  pressure << atmospheric_pressure - pressure_sensor_in_nav.z() * gravity * water_density;
  return pressure;
}

} // namespace uwv_kalman_filters

#endif
