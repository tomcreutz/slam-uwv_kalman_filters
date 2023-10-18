#ifndef UWV_KALMAN_FILTERS_POSEUKF_HPP
#define UWV_KALMAN_FILTERS_POSEUKF_HPP

#include "pose_estimation/UnscentedKalmanFilter.hpp"
#include "pose_estimation/Measurement.hpp"
#include "pose_estimation/GeographicProjection.hpp"
#include "PoseState.hpp"
#include "PoseUKFConfig.hpp"

namespace uwv_kalman_filters
{

class PoseUKF : public pose_estimation::UnscentedKalmanFilter<SimplePoseState>
{
  /**
 * This implements a full model aided inertial localization solution for
 * autonomous underwater vehicles.
 *
 * As minimal input the filter relays on rotation rates and accelerations from
 * an IMU and velocities from a DVL. Given force and torque measurements an AUV
 * motion model aids the velocity estimate during DVL drop outs. ADCP
 * measurements further aid the estimation in cases of DVL bottom-lock loss.
 * Given gyroscopes capable of sensing the rotation of the earth (e.g. a fiber
 * optic gyro) this filter is able to estimate it's true heading.
 *
 * NOTE: In this filter the IMU frame is, in order to keep a certain algorithmic
 * simplicity, not considered to be rotated with respect to the body frame.
 * Rotation rates and acceleration, as well as the corresponding configuration
 * parameters therefore would need to be rotated to the body frame before
 * integrating them in this filter. Due to the used grographic projection the
 * navigation frame is in NWU (North-West-Up).
 */

public:
/* Measurements of the filter */
  MEASUREMENT(Pressure, 1)
  MEASUREMENT(RotationRate, 3)
  MEASUREMENT(Acceleration, 3)
  MEASUREMENT(Velocity, 3)

  /**
   * Initializes the filter from a given initial pose (IMU in NWU-navigation
   * frame) and pose uncertainty. The rest of the state and state covariance is
   * computed from the parameters in PoseUKFConfig.
   */
  PoseUKF(
    const Eigen::Vector3d & imu_in_nwu_pos,
    const Eigen::Matrix3d & imu_in_nwu_pos_cov,
    const Eigen::Quaterniond & imu_in_nwu_rot,
    const Eigen::Matrix3d & imu_in_nwu_rot_cov,
    const PoseUKFConfig & pose_filter_config);

  /**
   * Initializes the filter from a given initial state and state uncertainty.
   * @param initial_state full filter state
   * @param state_cov initial state uncertainty
   * @param location geographic location (this is used as the reference frame of
   * geographic projection)
   * @param filter_parameter persistent parametes of the filter
   */
  PoseUKF(
    const State & initial_state, const Covariance & state_cov,
    const PoseUKFConfig & pose_filter_config);

  virtual ~PoseUKF() {}


  /**
   * Measurement Integration functions
  */

  /* Pressure in liquid in pascal (N/m²) */
  void integrateMeasurement(const Pressure & pressure);

  /* Rotation rates of IMU expressed in the IMU frame */
  void integrateMeasurement(const RotationRate & rotation_rate);

  /* Accelerations of IMU expressed in the IMU frame */
  void integrateMeasurement(const Acceleration & acceleration);

  /* Linear Velocities expressed in the IMU frame */
  void integrateMeasurement(const Velocity & velocity);

  /*
  * Utils
  */

  /**
    * Sets the process noise covariance from the filter specific configuration.
    * @param pose_filter_config filter specific configuration
    * @param imu_delta_t delta time between IMU measurements in seconds (e.g.
    * 0.01 @ 100Hz)
    * @param imu_in_body IMU with respect to body frame of the robot.
    *                    If no value is given it is assumed to be equal.
    * (optional)
    */
  void setProcessNoiseFromConfig(const PoseUKFConfig & pose_filter_config, double imu_delta_t);

  /* Resets filter with an externally provided Pose*/
  void resetFilterWithExternalPose(const Eigen::Affine3d & imu_in_nwu);

  RotationRate::Mu getRotationRate();

protected:
  void predictionStepImpl(double delta_t);

  std::shared_ptr<pose_estimation::GeographicProjection> projection_;
  PoseUKFConfig config_;
  RotationRate::Mu rotation_rate_;

  double water_density_offset_;
  Eigen::Vector3d gravity_;

  /* nanoseconds since the initialization of the filter */
  std::chrono::nanoseconds filter_ts_;
};

} // namespace auv_localization

#endif
