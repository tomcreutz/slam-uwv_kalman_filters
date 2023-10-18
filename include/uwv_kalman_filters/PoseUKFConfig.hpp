#ifndef UWV_KALMAN_FILTERS_POSE_UKF_CONFIG_HPP
#define UWV_KALMAN_FILTERS_POSE_UKF_CONFIG_HPP

#include <Eigen/Geometry>
#include <vector>


namespace uwv_kalman_filters
{
struct InertialNoiseParameters
{
  /*  Random walk ((m/s^s)/sqrt(Hz) for accelerometers or (rad/s)/sqrt(Hz) for gyros) */
  Eigen::Vector3d randomwalk;

  /*  Bias offset in static regimen (initial bias value) */
  Eigen::Vector3d bias_offset;

  /* Bias instability (m/s^2 for accelerometers or rad/s for gyros) */
  Eigen::Vector3d bias_instability;

  /* Tau value to limit the bias gain in seconds */
  double bias_tau;
};

struct LocationConfiguration
{
  /* Latitude in radians */
  double latitude;

  /* Longitude in radians */
  double longitude;

  /* Altitude in meters */
  double altitude;
};

struct HydrostaticConfiguration
{
  /* Density of water in kg/m³ */
  double water_density;
  /* Water density standard deviation */
  double water_density_limits;
  /* Time scale for water density change in seconds */
  double water_density_tau;
  /* atmospheric pressure in pascal (N/m²) */
  double atmospheric_pressure;
  /* Standard deviation of pressure measurements in N/m²sqrt(Hz) */
  double pressure_std;
};

struct PoseUKFConfig
{
  /* Inerial noise parameters for acceleration */
  InertialNoiseParameters acceleration;

  /** Inerial noise parameters for acceleration */
  InertialNoiseParameters rotation_rate;

  /* Latitude and Longitude of operational area */
  LocationConfiguration location;

  /** Max change of acceleration in m/s^3 */
  Eigen::Vector3d max_jerk;
  /* keep those static or add water density and atmospheric pressure to state ?*/
  HydrostaticConfiguration hydrostatics;

  /*
  * Transformations that will be needed
  */

  Eigen::Affine3d imu_in_body;
  Eigen::Affine3d dvl_in_imu;
  Eigen::Affine3d pressure_in_imu;


};

}
#endif
