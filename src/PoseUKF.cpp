#include "PoseUKF.hpp"
#include "pose_estimation/GravitationalModel.hpp"

#include "measurement_models/pressure_sensor.hpp"
#include "measurement_models/acceleration.hpp"
#include "measurement_models/linear_velocity.hpp"

#include "utils/mahalanobis.hpp"

using namespace uwv_kalman_filters;

// todo(tcreutz): Should I move this function into a seperate file, which is included ?
// process model
template<typename FilterState>
FilterState processModel(
  const FilterState & state, const Eigen::Vector3d & rotation_rate,
  std::shared_ptr<pose_estimation::GeographicProjection> projection,
  const double & gyro_bias_tau,
  const Eigen::Vector3d & gyro_bias_offset,
  const double & acc_bias_tau,
  const Eigen::Vector3d & acc_bias_offset,
  double delta_time)
{
  FilterState new_state(state);

  // apply velocity
  new_state.position.boxplus(state.velocity, delta_time);

  // apply angular velocity
  double latitude, longitude;
  projection->navToWorld(
    state.position.x(), state.position.y(), latitude,
    longitude);
  Eigen::Vector3d earth_rotation =
    Eigen::Vector3d(
    pose_estimation::EARTHW * cos(latitude), 0.,
    pose_estimation::EARTHW * sin(latitude));
  Eigen::Vector3d angular_velocity =
    state.orientation * (rotation_rate - state.bias_gyro) - earth_rotation;
  new_state.orientation.boxplus(angular_velocity, delta_time);

  // apply acceleration
  new_state.velocity.boxplus(state.acceleration, delta_time);

  Eigen::Vector3d gyro_bias_delta =
    (-1.0 / gyro_bias_tau) *
    (Eigen::Vector3d(state.bias_gyro) - gyro_bias_offset);
  new_state.bias_gyro.boxplus(gyro_bias_delta, delta_time);

  Eigen::Vector3d acc_bias_delta =
    (-1.0 / acc_bias_tau) *
    (Eigen::Vector3d(state.bias_acc) - acc_bias_offset);
  new_state.bias_acc.boxplus(acc_bias_delta, delta_time);

  return new_state;
}


PoseUKF::PoseUKF(
  const Eigen::Vector3d & imu_in_nwu_pos,
  const Eigen::Matrix3d & imu_in_nwu_pos_cov,
  const Eigen::Quaterniond & imu_in_nwu_rot,
  const Eigen::Matrix3d & imu_in_nwu_rot_cov,
  const PoseUKFConfig & pose_filter_config)
: config_(pose_filter_config)
{
  // timestamp is in nanoseconds
  // todo(tcreutz): Should I also make pose_estimation timestamps to nanoseconds ?
  filter_ts_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::nanoseconds(0));


  State initial_state;
  initial_state.position = TranslationType(imu_in_nwu_pos);
  initial_state.orientation = RotationType(imu_in_nwu_rot);
  initial_state.velocity = VelocityType(Eigen::Vector3d::Zero());
  initial_state.acceleration = AccelerationType(Eigen::Vector3d::Zero());
  initial_state.bias_gyro = BiasType(
    config_.imu_in_body.rotation() *
    config_.rotation_rate.bias_offset);
  initial_state.bias_acc =
    BiasType(config_.imu_in_body.rotation() * config_.acceleration.bias_offset);
  gravity_ = Eigen::Vector3d::Zero();
  gravity_(2) = pose_estimation::GravitationalModel::WGS_84(
    config_.location.latitude, config_.location.altitude);

  Covariance initial_state_cov = Covariance::Zero();
  MTK::subblock(initial_state_cov, &State::position) = imu_in_nwu_pos_cov;
  MTK::subblock(initial_state_cov, &State::orientation) = imu_in_nwu_rot_cov;
  MTK::subblock(initial_state_cov, &State::velocity) =
    Eigen::Matrix3d::Identity();    // velocity is unknown at the start
  MTK::subblock(initial_state_cov, &State::acceleration) =
    10 * Eigen::Matrix3d::Identity();    // acceleration is unknown at the start
  MTK::subblock(initial_state_cov, &State::bias_gyro) =
    config_.imu_in_body.rotation() *
    config_.rotation_rate.bias_instability.cwiseAbs2().asDiagonal() *
    config_.imu_in_body.rotation().transpose();
  MTK::subblock(initial_state_cov, &State::bias_acc) =
    config_.imu_in_body.rotation() *
    config_.acceleration.bias_instability.cwiseAbs2().asDiagonal() *
    config_.imu_in_body.rotation().transpose();

  initializeFilter(initial_state, initial_state_cov);

  // TODO: does it even make sense to track this variable ?
  water_density_offset_ = config_.hydrostatics.water_density;

  rotation_rate_ = RotationRate::Mu::Zero();

  projection_.reset(
    new pose_estimation::GeographicProjection(
      config_.location.latitude, config_.location.longitude));
}

PoseUKF::PoseUKF(
  const State & initial_state,
  const Covariance & state_cov,
  const PoseUKFConfig & pose_filter_config)
: config_(pose_filter_config)
{
  // timestamp is in microseconds
  // todo(tcreutz): Should I also make pose_estimation timestamps to nanoseconds ?
  filter_ts_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::nanoseconds(0));

  initializeFilter(initial_state, state_cov);

  // TODO: does it even make sense to track this variable ?
  water_density_offset_ = config_.hydrostatics.water_density;

  rotation_rate_ = RotationRate::Mu::Zero();

  projection_.reset(
    new pose_estimation::GeographicProjection(
      config_.location.latitude, config_.location.longitude));
}

/**
 * Measurement Integration Functions
*/

void PoseUKF::integrateMeasurement(
  const Pressure & pressure)
{
  checkMeasurment(pressure.mu, pressure.cov);
  ukf->update(
    pressure.mu,
    boost::bind(
      measurementPressureSensor<State>, _1, config_.pressure_in_imu.translation(),
      config_.hydrostatics.atmospheric_pressure, config_.hydrostatics.water_density, gravity_(2)),
    boost::bind(ukfom::id<Pressure::Cov>, pressure.cov),
    ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void PoseUKF::integrateMeasurement(const Velocity & velocity)
{
  checkMeasurment(velocity.mu, velocity.cov);
  ukf->update(
    velocity.mu, boost::bind(measurementVelocity<State>, _1),
    boost::bind(ukfom::id<Velocity::Cov>, velocity.cov),
    ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void PoseUKF::integrateMeasurement(const Acceleration & acceleration)
{
  checkMeasurment(acceleration.mu, acceleration.cov);
  ukf->update(
    acceleration.mu, boost::bind(measurementAcceleration<State>, _1, gravity_),
    boost::bind(ukfom::id<Acceleration::Cov>, acceleration.cov),
    ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void PoseUKF::integrateMeasurement(const RotationRate & rotation_rate)
{
  checkMeasurment(rotation_rate.mu, rotation_rate.cov);
  this->rotation_rate_ = rotation_rate.mu;
}

void PoseUKF::predictionStepImpl(double delta_t)
{
  Eigen::Matrix3d rot = ukf->mu().orientation.matrix();
  Covariance process_noise = process_noise_cov;
  // uncertainty matrix calculations
  MTK::subblock(process_noise, &State::orientation) =
    rot * MTK::subblock(process_noise_cov, &State::orientation) *
    rot.transpose();

  Eigen::Vector3d scaled_velocity = ukf->mu().velocity;
  scaled_velocity[2] =
    10 * scaled_velocity[2];    // scale Z velocity to have 10x more impact

  process_noise = pow(delta_t, 2.) * process_noise;

  ukf->predict(
    boost::bind(
      processModel<WState>, _1, rotation_rate_, projection_,
      config_.rotation_rate.bias_tau, config_.rotation_rate.bias_offset,
      config_.acceleration.bias_tau, config_.acceleration.bias_offset, delta_t),
    MTK_UKF::cov(process_noise));
}

/**
* Utils
*/

//todo(tcreutz): Do I even need this function as it is ?
void PoseUKF::setProcessNoiseFromConfig(
  const PoseUKFConfig & filter_config, double imu_delta_t)
{
  Eigen::Matrix3d imu_in_body_m = filter_config.imu_in_body.rotation();

  Covariance process_noise_cov = PoseUKF::Covariance::Zero();
  // Euler integration error position: (1/6/4 * jerk_max * dt^3)^2
  // assuming max jerk is 4*sigma devide by 4
  MTK::subblock(process_noise_cov, &State::position) =
    1.5 * (std::pow(imu_delta_t, 4.0) *
    ((1. / 6.) * 0.25 * filter_config.max_jerk).cwiseAbs2())
    .asDiagonal();

  // Euler integration error velocity: (1/2/4 * jerk_max * dt^2)^2
  MTK::subblock(process_noise_cov, &State::velocity) =
    1.5 * (std::pow(imu_delta_t, 2.0) *
    (0.5 * 0.25 * filter_config.max_jerk).cwiseAbs2())
    .asDiagonal();

  // Euler integration error acceleration: (1/4 * jerk_max * dt)^2
  MTK::subblock(process_noise_cov, &State::acceleration) =
    (0.25 * filter_config.max_jerk).cwiseAbs2().asDiagonal();

  MTK::subblock(process_noise_cov, &State::orientation) =
    imu_in_body_m *
    filter_config.rotation_rate.randomwalk.cwiseAbs2().asDiagonal() *
    imu_in_body_m.transpose();
  MTK::subblock(process_noise_cov, &State::bias_gyro) =
    imu_in_body_m *
    (2. / (filter_config.rotation_rate.bias_tau * imu_delta_t)) *
    filter_config.rotation_rate.bias_instability.cwiseAbs2().asDiagonal() *
    imu_in_body_m.transpose();
  MTK::subblock(process_noise_cov, &State::bias_acc) =
    imu_in_body_m *
    (2. / (filter_config.acceleration.bias_tau * imu_delta_t)) *
    filter_config.acceleration.bias_instability.cwiseAbs2().asDiagonal() *
    imu_in_body_m.transpose();

  setProcessNoiseCovariance(process_noise_cov);
}

void PoseUKF::resetFilterWithExternalPose(
  const Eigen::Affine3d & imu_in_nwu)
{
  State new_state = ukf->mu();
  new_state.position = TranslationType(imu_in_nwu.translation());
  new_state.orientation = RotationType(MTK::SO3<double>(imu_in_nwu.rotation()));
  ukf.reset(
    new MTK_UKF(
      new_state,
      ukf->sigma()));  // elaborate whether sigma can / has to be reset or
                       // changed or if it can stay like this
}

PoseUKF::RotationRate::Mu PoseUKF::getRotationRate()
{
  double latitude, longitude;
  projection_->navToWorld(
    ukf->mu().position.x(), ukf->mu().position.y(),
    latitude, longitude);
  Eigen::Vector3d earth_rotation =
    Eigen::Vector3d(
    pose_estimation::EARTHW * cos(latitude), 0.,
    pose_estimation::EARTHW * sin(latitude));
  return rotation_rate_ - ukf->mu().bias_gyro -
         ukf->mu().orientation.inverse() * earth_rotation;
}
