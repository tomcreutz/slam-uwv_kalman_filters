#include "SimplePoseUKF.hpp"
#include <math.h>
#include <pose_estimation/GravitationalModel.hpp>
#include <pose_estimation/GeographicProjection.hpp>
#include <mtk/types/S2.hpp>

using namespace uwv_kalman_filters;

// process model
template <typename FilterState>
FilterState
processModel(const FilterState &state, const Eigen::Vector3d &rotation_rate,
             std::shared_ptr<pose_estimation::GeographicProjection> projection,
             const double water_density_offset,
             const SimplePoseUKF::PoseUKFParameter &filter_parameter, double delta_time)
{
  FilterState new_state(state);

  // apply velocity
  new_state.position.boxplus(state.velocity, delta_time);

  // apply angular velocity
  double latitude, longitude;
  projection->navToWorld(state.position.x(), state.position.y(), latitude, longitude);
  Eigen::Vector3d earth_rotation = Eigen::Vector3d(pose_estimation::EARTHW * cos(latitude), 0., pose_estimation::EARTHW * sin(latitude));
  Eigen::Vector3d angular_velocity = state.orientation * (rotation_rate - state.bias_gyro) - earth_rotation;
  new_state.orientation.boxplus(angular_velocity, delta_time);

  // apply acceleration
  new_state.velocity.boxplus(state.acceleration, delta_time);

  Eigen::Vector3d gyro_bias_delta = (-1.0 / filter_parameter.gyro_bias_tau) *
                                    (Eigen::Vector3d(state.bias_gyro) - filter_parameter.gyro_bias_offset);
  new_state.bias_gyro.boxplus(gyro_bias_delta, delta_time);

  Eigen::Vector3d acc_bias_delta = (-1.0 / filter_parameter.acc_bias_tau) *
                                   (Eigen::Vector3d(state.bias_acc) - filter_parameter.acc_bias_offset);
  new_state.bias_acc.boxplus(acc_bias_delta, delta_time);

  WaterVelocityType::vectorized_type water_velocity_delta = (-1.0 / filter_parameter.water_velocity_tau) *
                                                            (Eigen::Map<const WaterVelocityType::vectorized_type>(state.water_velocity.data()));
  new_state.water_velocity.boxplus(water_velocity_delta, delta_time);

  WaterVelocityType::vectorized_type water_velocity_below_delta = (-1.0 / filter_parameter.water_velocity_tau) *
                                                                  (Eigen::Map<const WaterVelocityType::vectorized_type>(state.water_velocity_below.data()));
  new_state.water_velocity_below.boxplus(water_velocity_below_delta, delta_time);

  WaterVelocityType::vectorized_type bias_adcp_delta = (-1.0 / filter_parameter.adcp_bias_tau) *
                                                       (Eigen::Map<const WaterVelocityType::vectorized_type>(state.bias_adcp.data()));
  new_state.bias_adcp.boxplus(bias_adcp_delta, delta_time);

  DensityType::vectorized_type water_density_delta;
  water_density_delta << (-1.0 / filter_parameter.water_density_tau) * (state.water_density(0) - water_density_offset);
  new_state.water_density.boxplus(water_density_delta, delta_time);

  return new_state;
}

// measurement models
template <typename FilterState>
Eigen::Matrix<TranslationType::scalar, 2, 1>
measurementXYPosition(const FilterState &state)
{
  return state.position.block(0, 0, 2, 1);
}

Eigen::Matrix<TranslationType::scalar, 2, 1>
measurementDelayedXYPosition(const Eigen::Vector2d &delayed_position)
{
  return delayed_position;
}

template <typename FilterState>
Eigen::Matrix<TranslationType::scalar, 1, 1>
measurementZPosition(const FilterState &state)
{
  return state.position.block(2, 0, 1, 1);
}

template <typename FilterState>
Eigen::Matrix<TranslationType::scalar, 1, 1>
measurementPressureSensor(const FilterState &state, const Eigen::Vector3d &pressure_sensor_in_imu, double atmospheric_pressure)
{
  Eigen::Vector3d pressure_sensor_in_nav = state.position + state.orientation * pressure_sensor_in_imu;
  Eigen::Matrix<TranslationType::scalar, 1, 1> pressure;
  pressure << atmospheric_pressure - pressure_sensor_in_nav.z() * state.gravity(0) * state.water_density(0);
  return pressure;
}

template <typename FilterState>
VelocityType
measurementVelocity(const FilterState &state)
{
  // return expected velocities in the IMU frame
  return VelocityType(state.orientation.inverse() * state.velocity);
}

template <typename FilterState>
AccelerationType
measurementAcceleration(const FilterState &state)
{
  // returns expected accelerations in the IMU frame
  return AccelerationType(state.orientation.inverse() * (Eigen::Vector3d(state.acceleration) + Eigen::Vector3d(0., 0., state.gravity(0))) + Eigen::Vector3d(state.bias_acc));
}

template <typename FilterState>
WaterVelocityType
measurementWaterCurrents(const FilterState &state, double cell_weighting)
{
  // returns expected water current measurements in the IMU frame
  Eigen::Vector3d water_velocity_below;
  water_velocity_below << state.water_velocity_below[0], state.water_velocity_below[1], 0;
  water_velocity_below = state.orientation.inverse() * ((Eigen::Vector3d)state.velocity - water_velocity_below);

  Eigen::Vector3d water_velocity;
  water_velocity << state.water_velocity[0], state.water_velocity[1], 0;
  water_velocity = state.orientation.inverse() * ((Eigen::Vector3d)state.velocity - water_velocity);

  WaterVelocityType expected_measurement;
  expected_measurement[0] = cell_weighting * water_velocity_below[0] + (1 - cell_weighting) * water_velocity[0] + state.bias_adcp[0];
  expected_measurement[1] = cell_weighting * water_velocity_below[1] + (1 - cell_weighting) * water_velocity[1] + state.bias_adcp[1];

  return expected_measurement;
}

/**
 * Augments the pose filter state with a marker pose.
 * This allows to take the uncertainty of the marker pose into account.
 */
MTK_BUILD_MANIFOLD(SimplePoseStateWithMarker,
                   ((ukfom::mtkwrap<SimplePoseState>, filter_state))((TranslationType, marker_position)) // position of a marker in navigation frame
                   ((RotationType, marker_orientation))                                                  // orientation of a marker in navigation frame
)
typedef ukfom::mtkwrap<SimplePoseStateWithMarker> WSimplePoseStateWithMarker;
typedef Eigen::Matrix<SimplePoseStateWithMarker::scalar, SimplePoseStateWithMarker::DOF, SimplePoseStateWithMarker::DOF> SimplePoseStateWithMarkerCov;
typedef ukfom::mtkwrap<MTK::S2<double>> WS2Type;

template <typename FilterState>
WS2Type
measurementVisualLandmark(const FilterState &state, const Eigen::Vector3d &feature_pos, const Eigen::Affine3d &cam_in_imu)
{
  Eigen::Affine3d imu_in_nav = Eigen::Affine3d(state.filter_state.orientation);
  imu_in_nav.translation() = state.filter_state.position;
  // imu_in_nav.linear() = state.filter_state.orientation.toRotationMatrix();
  Eigen::Affine3d nav_in_cam = (imu_in_nav * cam_in_imu).inverse();
  Eigen::Vector3d feature_in_cam = nav_in_cam * (state.marker_orientation * feature_pos + state.marker_position);
  return WS2Type(MTK::S2<double>(feature_in_cam));
}

/**
 * Augments the pose filter state with a delayed position.
 */
MTK_BUILD_MANIFOLD(SimplePoseStateWithDelayedPosition,
                   ((ukfom::mtkwrap<SimplePoseState>, filter_state))((Translation2DType, delayed_position)) // delayed position in navigation frame
)
typedef ukfom::mtkwrap<SimplePoseStateWithDelayedPosition> WSimplePoseStateWithDelayedPosition;
typedef Eigen::Matrix<SimplePoseStateWithDelayedPosition::scalar, SimplePoseStateWithDelayedPosition::DOF, SimplePoseStateWithDelayedPosition::DOF> SimplePoseStateWithDelayedPositionCov;

template <typename FilterState>
Translation2DType
measurementDelayedPosition(const FilterState &state)
{
  return state.delayed_position;
}

// functions for innovation gate test, using mahalanobis distance
template <typename scalar_type>
static bool d2p99(const scalar_type &mahalanobis2)
{
  if (mahalanobis2 > 9.21) // for 2 degrees of freedom, 99% likelihood = 9.21, https://www.uam.es/personal_pdi/ciencias/anabz/Prest/Trabajos/Critical_Values_of_the_Chi-Squared_Distribution.pdf
  {
    return false;
  }
  else
  {
    return true;
  }
}

template <typename scalar_type>
static bool d2p95(const scalar_type &mahalanobis2)
{
  if (mahalanobis2 > 5.991) // for 2 degrees of freedom, 95% likelihood = 5.991, https://www.uam.es/personal_pdi/ciencias/anabz/Prest/Trabajos/Critical_Values_of_the_Chi-Squared_Distribution.pdf
  {
    return false;
  }
  else
  {
    return true;
  }
}

SimplePoseUKF::SimplePoseUKF(const Eigen::Vector3d &imu_in_nwu_pos, const Eigen::Matrix3d &imu_in_nwu_pos_cov,
                             const Eigen::Quaterniond &imu_in_nwu_rot, const Eigen::Matrix3d &imu_in_nwu_rot_cov,
                             const PoseUKFConfig &filter_config,
                             const Eigen::Affine3d &imu_in_body) : filter_ts(0)
{
  State initial_state;
  initial_state.position = TranslationType(imu_in_nwu_pos);
  initial_state.orientation = RotationType(imu_in_nwu_rot);
  initial_state.velocity = VelocityType(Eigen::Vector3d::Zero());
  initial_state.acceleration = AccelerationType(Eigen::Vector3d::Zero());
  initial_state.bias_gyro = BiasType(imu_in_body.rotation() * filter_config.rotation_rate.bias_offset);
  initial_state.bias_acc = BiasType(imu_in_body.rotation() * filter_config.acceleration.bias_offset);
  Eigen::Matrix<double, 1, 1> gravity;
  gravity(0) = pose_estimation::GravitationalModel::WGS_84(filter_config.location.latitude, filter_config.location.altitude);
  initial_state.gravity = GravityType(gravity);
  initial_state.water_velocity = WaterVelocityType(Eigen::Vector2d::Zero());
  initial_state.water_velocity_below = WaterVelocityType(Eigen::Vector2d::Zero());
  initial_state.bias_adcp = WaterVelocityType(Eigen::Vector2d::Zero());
  Eigen::Matrix<double, 1, 1> water_density;
  water_density << filter_config.hydrostatics.water_density;
  initial_state.water_density = DensityType(water_density);
  // initial_state.delayed_position = Translation2DType(imu_in_nwu_pos.head<2>());

  Covariance initial_state_cov = Covariance::Zero();
  MTK::subblock(initial_state_cov, &State::position) = imu_in_nwu_pos_cov;
  MTK::subblock(initial_state_cov, &State::orientation) = imu_in_nwu_rot_cov;
  MTK::subblock(initial_state_cov, &State::velocity) = Eigen::Matrix3d::Identity();          // velocity is unknown at the start
  MTK::subblock(initial_state_cov, &State::acceleration) = 10 * Eigen::Matrix3d::Identity(); // acceleration is unknown at the start
  MTK::subblock(initial_state_cov, &State::bias_gyro) = imu_in_body.rotation() * filter_config.rotation_rate.bias_instability.cwiseAbs2().asDiagonal() * imu_in_body.rotation().transpose();
  MTK::subblock(initial_state_cov, &State::bias_acc) = imu_in_body.rotation() * filter_config.acceleration.bias_instability.cwiseAbs2().asDiagonal() * imu_in_body.rotation().transpose();
  Eigen::Matrix<double, 1, 1> gravity_var;
  gravity_var << pow(0.05, 2.); // give the gravity model a sigma of 5 cm/s^2 at the start
  MTK::subblock(initial_state_cov, &State::gravity) = gravity_var;
  MTK::subblock(initial_state_cov, &State::water_velocity) = pow(filter_config.water_velocity.limits, 2) * Eigen::Matrix2d::Identity();
  MTK::subblock(initial_state_cov, &State::water_velocity_below) = pow(filter_config.water_velocity.limits, 2) * Eigen::Matrix2d::Identity();
  MTK::subblock(initial_state_cov, &State::bias_adcp) = pow(filter_config.water_velocity.adcp_bias_limits, 2) * Eigen::Matrix2d::Identity();
  Eigen::Matrix<double, 1, 1> water_density_var;
  water_density_var << pow(filter_config.hydrostatics.water_density_limits, 2.);
  MTK::subblock(initial_state_cov, &State::water_density) = water_density_var;
  // MTK::subblock(initial_state_cov, &State::delayed_position) = MTK::subblock(initial_state_cov, &State::position).block(0, 0, 2, 2);

  initializeFilter(initial_state, initial_state_cov);

  water_density_offset = initial_state.water_density(0);

  rotation_rate = RotationRate::Mu::Zero();

  projection.reset(new pose_estimation::GeographicProjection(filter_config.location.latitude, filter_config.location.longitude));

  filter_parameter.imu_in_body = imu_in_body.translation();
  filter_parameter.acc_bias_tau = filter_config.acceleration.bias_tau;
  filter_parameter.acc_bias_offset = imu_in_body.rotation() * filter_config.acceleration.bias_offset;
  filter_parameter.gyro_bias_tau = filter_config.rotation_rate.bias_tau;
  filter_parameter.gyro_bias_offset = imu_in_body.rotation() * filter_config.rotation_rate.bias_offset;
  filter_parameter.water_velocity_tau = filter_config.water_velocity.tau;
  filter_parameter.water_velocity_limits = filter_config.water_velocity.limits;
  filter_parameter.water_velocity_scale = filter_config.water_velocity.scale;
  filter_parameter.adcp_bias_tau = filter_config.water_velocity.adcp_bias_tau;
  filter_parameter.atmospheric_pressure = filter_config.hydrostatics.atmospheric_pressure;
  filter_parameter.water_density_tau = filter_config.hydrostatics.water_density_tau;
}

SimplePoseUKF::SimplePoseUKF(const State &initial_state, const Covariance &state_cov,
                             const LocationConfiguration &location,
                             const PoseUKFParameter &filter_parameter) : filter_parameter(filter_parameter), filter_ts(0)
{
  initializeFilter(initial_state, state_cov);

  rotation_rate = RotationRate::Mu::Zero();

  water_density_offset = initial_state.water_density(0);

  projection.reset(new pose_estimation::GeographicProjection(location.latitude, location.longitude));
}

void SimplePoseUKF::setProcessNoiseFromConfig(const PoseUKFConfig &filter_config, double imu_delta_t,
                                              const Eigen::Quaterniond &imu_in_body)
{
  Eigen::Matrix3d imu_in_body_m = imu_in_body.toRotationMatrix();

  Covariance process_noise_cov = SimplePoseUKF::Covariance::Zero();
  // Euler integration error position: (1/6/4 * jerk_max * dt^3)^2
  // assuming max jerk is 4*sigma devide by 4
  MTK::subblock(process_noise_cov, &State::position) = 1.5 * (std::pow(imu_delta_t, 4.0) * ((1. / 6.) * 0.25 * filter_config.max_jerk).cwiseAbs2()).asDiagonal();

  // Euler integration error velocity: (1/2/4 * jerk_max * dt^2)^2
  MTK::subblock(process_noise_cov, &State::velocity) = 1.5 * (std::pow(imu_delta_t, 2.0) * (0.5 * 0.25 * filter_config.max_jerk).cwiseAbs2()).asDiagonal();

  // Euler integration error acceleration: (1/4 * jerk_max * dt)^2
  MTK::subblock(process_noise_cov, &State::acceleration) = (0.25 * filter_config.max_jerk).cwiseAbs2().asDiagonal();

  MTK::subblock(process_noise_cov, &State::orientation) = imu_in_body_m * filter_config.rotation_rate.randomwalk.cwiseAbs2().asDiagonal() * imu_in_body_m.transpose();
  MTK::subblock(process_noise_cov, &State::bias_gyro) = imu_in_body_m * (2. / (filter_config.rotation_rate.bias_tau * imu_delta_t)) *
                                                        filter_config.rotation_rate.bias_instability.cwiseAbs2().asDiagonal() * imu_in_body_m.transpose();
  MTK::subblock(process_noise_cov, &State::bias_acc) = imu_in_body_m * (2. / (filter_config.acceleration.bias_tau * imu_delta_t)) *
                                                       filter_config.acceleration.bias_instability.cwiseAbs2().asDiagonal() * imu_in_body_m.transpose();
  Eigen::Matrix<double, 1, 1> gravity_noise;
  gravity_noise << 1.e-12; // add a tiny bit of noise only for numeric stability
  MTK::subblock(process_noise_cov, &State::gravity) = gravity_noise;

  MTK::subblock(process_noise_cov, &State::water_velocity) = (2. / (filter_config.water_velocity.tau * imu_delta_t)) *
                                                             pow(filter_config.water_velocity.limits, 2) * Eigen::Matrix2d::Identity();

  MTK::subblock(process_noise_cov, &State::water_velocity_below) = (2. / (filter_config.water_velocity.tau * imu_delta_t)) *
                                                                   pow(filter_config.water_velocity.limits, 2) * Eigen::Matrix2d::Identity();

  MTK::subblock(process_noise_cov, &State::bias_adcp) = (2. / (filter_config.water_velocity.adcp_bias_tau * imu_delta_t)) *
                                                        pow(filter_config.water_velocity.adcp_bias_limits, 2) * Eigen::Matrix2d::Identity();

  Eigen::Matrix<double, 1, 1> water_density_noise;
  water_density_noise << (2. / (filter_config.hydrostatics.water_density_tau * imu_delta_t)) * pow(filter_config.hydrostatics.water_density_limits, 2.);
  MTK::subblock(process_noise_cov, &State::water_density) = water_density_noise;
  // MTK::subblock(process_noise_cov, &State::delayed_position) = MTK::subblock(process_noise_cov, &State::position).block(0, 0, 2, 2);

  setProcessNoiseCovariance(process_noise_cov);
}

// void PoseUKF::setupDelayedStateBuffer(double maximum_delay)
// {
//     delayed_states.reset(new pose_estimation::DelayedStates<Translation2DType>(std::abs(maximum_delay)));
// }

void SimplePoseUKF::predictionStepImpl(double delta_t)
{
  Eigen::Matrix3d rot = ukf->mu().orientation.matrix();
  Covariance process_noise = process_noise_cov;
  // uncertainty matrix calculations
  MTK::subblock(process_noise, &State::orientation) = rot * MTK::subblock(process_noise_cov, &State::orientation) * rot.transpose();

  Eigen::Vector3d scaled_velocity = ukf->mu().velocity;
  scaled_velocity[2] = 10 * scaled_velocity[2]; // scale Z velocity to have 10x more impact

  MTK::subblock(process_noise, &State::water_velocity) = MTK::subblock(process_noise_cov, &State::water_velocity) + Eigen::Matrix<double, 2, 2>::Identity() * filter_parameter.water_velocity_scale * scaled_velocity.squaredNorm() * delta_t;

  MTK::subblock(process_noise, &State::water_velocity_below) = MTK::subblock(process_noise_cov, &State::water_velocity_below) + Eigen::Matrix<double, 2, 2>::Identity() * filter_parameter.water_velocity_scale * scaled_velocity.squaredNorm() * delta_t;

  process_noise = pow(delta_t, 2.) * process_noise;

  ukf->predict(boost::bind(processModel<WState>, _1, rotation_rate, projection, water_density_offset,
                           filter_parameter, delta_t),
               MTK_UKF::cov(process_noise));

  // save current state
  // if (delayed_states)
  // {
  //     filter_ts += pose_estimation::DelayedStates<Translation2DType>::fromSeconds(delta_t);
  //     auto sigma = ukf->sigma(); // has to be copied as MTK::subblock doesnt accept const
  //     delayed_states->pushState(filter_ts, ukf->mu().delayed_position, MTK::subblock(sigma, &State::delayed_position));
  // }
}

void SimplePoseUKF::integrateMeasurement(const Velocity &velocity)
{
  checkMeasurment(velocity.mu, velocity.cov);
  ukf->update(velocity.mu, boost::bind(measurementVelocity<State>, _1),
              boost::bind(ukfom::id<Velocity::Cov>, velocity.cov),
              ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void SimplePoseUKF::integrateMeasurement(const Acceleration &acceleration)
{
  checkMeasurment(acceleration.mu, acceleration.cov);
  ukf->update(acceleration.mu, boost::bind(measurementAcceleration<State>, _1),
              boost::bind(ukfom::id<Acceleration::Cov>, acceleration.cov),
              ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void SimplePoseUKF::integrateMeasurement(const RotationRate &rotation_rate)
{
  checkMeasurment(rotation_rate.mu, rotation_rate.cov);
  this->rotation_rate = rotation_rate.mu;
}

void SimplePoseUKF::integrateMeasurement(const Z_Position &z_position)
{
  checkMeasurment(z_position.mu, z_position.cov);
  ukf->update(z_position.mu, boost::bind(measurementZPosition<State>, _1),
              boost::bind(ukfom::id<Z_Position::Cov>, z_position.cov),
              ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void SimplePoseUKF::integrateMeasurement(const XY_Position &xy_position)
{
  checkMeasurment(xy_position.mu, xy_position.cov);
  ukf->update(xy_position.mu, boost::bind(measurementXYPosition<State>, _1),
              boost::bind(ukfom::id<XY_Position::Cov>, xy_position.cov),
              ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void SimplePoseUKF::integrateDelayedPositionMeasurement(const XY_Position &xy_position, const Eigen::Vector2d &delayed_position)
{
  checkMeasurment(xy_position.mu, xy_position.cov);
  ukf->update(xy_position.mu, boost::bind(measurementDelayedXYPosition, delayed_position),
              boost::bind(ukfom::id<XY_Position::Cov>, xy_position.cov),
              ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void SimplePoseUKF::integrateDelayedPositionMeasurementWithStateAugmentation(const XY_Position &xy_position, const Eigen::Vector2d &delayed_position, const Eigen::Matrix2d &cov_delayed_position)
{
  checkMeasurment(xy_position.mu, xy_position.cov);
  // Augment the filter state with the marker pose
  WSimplePoseStateWithDelayedPosition augmented_state;
  augmented_state.filter_state = ukf->mu();
  augmented_state.delayed_position = Translation2DType(delayed_position);
  SimplePoseStateWithDelayedPositionCov augmented_state_cov = SimplePoseStateWithDelayedPositionCov::Zero();
  augmented_state_cov.block(0, 0, WState::DOF, WState::DOF) = ukf->sigma();
  augmented_state_cov.bottomRightCorner<2, 2>() = cov_delayed_position;
  ukfom::ukf<WSimplePoseStateWithDelayedPosition> augmented_ukf(augmented_state, augmented_state_cov);
  augmented_ukf.update(xy_position.mu, boost::bind(measurementDelayedPosition<WSimplePoseStateWithDelayedPosition>, _1),
                       boost::bind(ukfom::id<XY_Position::Cov>, xy_position.cov),
                       ukfom::accept_any_mahalanobis_distance<State::scalar>);
  ukf.reset(new MTK_UKF(augmented_ukf.mu().filter_state, augmented_ukf.sigma().block(0, 0, WState::DOF, WState::DOF)));
}

void SimplePoseUKF::integrateMeasurement(const Pressure &pressure, const Eigen::Vector3d &pressure_sensor_in_imu)
{
  checkMeasurment(pressure.mu, pressure.cov);
  ukf->update(pressure.mu, boost::bind(measurementPressureSensor<State>, _1, pressure_sensor_in_imu, filter_parameter.atmospheric_pressure),
              boost::bind(ukfom::id<Pressure::Cov>, pressure.cov),
              ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void SimplePoseUKF::integrateMeasurement(const GeographicPosition &geo_position, const Eigen::Vector3d &gps_in_body)
{
  checkMeasurment(geo_position.mu, geo_position.cov);

  // project geographic position to local NWU plane
  Eigen::Matrix<TranslationType::scalar, 2, 1> projected_position;
  projection->worldToNav(geo_position.mu.x(), geo_position.mu.y(), projected_position.x(), projected_position.y());
  projected_position = projected_position - (ukf->mu().orientation * gps_in_body).head<2>();

  ukf->update(projected_position, boost::bind(measurementXYPosition<State>, _1),
              boost::bind(ukfom::id<XY_Position::Cov>, geo_position.cov),
              d2p95<State::scalar>);
}

void SimplePoseUKF::integrateMeasurement(const WaterVelocityMeasurement &adcp_measurements, double cell_weighting)
{
  checkMeasurment(adcp_measurements.mu, adcp_measurements.cov);

  ukf->update(adcp_measurements.mu, boost::bind(measurementWaterCurrents<State>, _1, cell_weighting),
              boost::bind(ukfom::id<WaterVelocityMeasurement::Cov>, adcp_measurements.cov),
              d2p95<State::scalar>);
}

void SimplePoseUKF::integrateMeasurement(const std::vector<VisualFeatureMeasurement> &marker_corners,
                                         const std::vector<Eigen::Vector3d> &feature_positions,
                                         const Eigen::Affine3d &marker_pose, const Eigen::Matrix<double, 6, 6> cov_marker_pose,
                                         const CameraConfiguration &camera_config, const Eigen::Affine3d &camera_in_IMU)
{
  // Augment the filter state with the marker pose
  WSimplePoseStateWithMarker augmented_state;
  augmented_state.filter_state = ukf->mu();
  augmented_state.marker_position = TranslationType(marker_pose.translation());
  augmented_state.marker_orientation = RotationType(MTK::SO3<double>(marker_pose.rotation()));
  SimplePoseStateWithMarkerCov augmented_state_cov = SimplePoseStateWithMarkerCov::Zero();
  augmented_state_cov.block(0, 0, WState::DOF, WState::DOF) = ukf->sigma();
  augmented_state_cov.bottomRightCorner<6, 6>() = cov_marker_pose;
  ukfom::ukf<WSimplePoseStateWithMarker> augmented_ukf(augmented_state, augmented_state_cov);

  double fx2 = std::pow(camera_config.fx, 2.);
  double fy2 = std::pow(camera_config.fy, 2.);
  double fxy = camera_config.fx * camera_config.fy;

  // Apply measurements on the augmented state
  for (unsigned i = 0; i < marker_corners.size() || i < feature_positions.size(); i++)
  {
    checkMeasurment(marker_corners[i].mu, marker_corners[i].cov);

    // project image coordinates into S2
    WS2Type projection(MTK::S2<double>((marker_corners[i].mu.x() - camera_config.cx) / camera_config.fx,
                                       (marker_corners[i].mu.y() - camera_config.cy) / camera_config.fy,
                                       1.0));
    std::cout << "Corner: " << i << " , Projection: " << projection << std::endl;
    Eigen::Matrix2d projection_cov;
    projection_cov << marker_corners[i].cov(0, 0) / fx2, marker_corners[i].cov(0, 1) / fxy,
        marker_corners[i].cov(1, 0) / fxy, marker_corners[i].cov(1, 1) / fy2;

    augmented_ukf.update(projection, boost::bind(measurementVisualLandmark<WSimplePoseStateWithMarker>, _1, feature_positions[i], camera_in_IMU),
                         boost::bind(ukfom::id<VisualFeatureMeasurement::Cov>, projection_cov),
                         ukfom::accept_any_mahalanobis_distance<WSimplePoseStateWithMarker::scalar>);
  }

  // Reconstructing the filter is currently the only way to modify the internal state of the filter

  ukf.reset(new MTK_UKF(augmented_ukf.mu().filter_state, augmented_ukf.sigma().block(0, 0, WState::DOF, WState::DOF)));
}

// bool PoseUKF::integrateDelayedMeasurement(const XY_Position &xy_position, double delay)
// {
//     checkMeasurment(xy_position.mu, xy_position.cov);
//     int64_t measurement_ts = filter_ts - pose_estimation::DelayedStates<Translation2DType>::fromSeconds(delay);
//     Translation2DType delayed_position;
//     pose_estimation::DelayedStates<Translation2DType>::Cov cov_delayed_position;
//     if (delayed_states && delayed_states->getClosestState(measurement_ts, delayed_position, cov_delayed_position))
//     {
//         // update current state with closest delayed state
//         Translation2DType current_delayed_position = ukf->mu().delayed_position;
//         WState delayed_state = ukf->mu();
//         delayed_state.delayed_position = delayed_position;
//         ukf.reset(new MTK_UKF(delayed_state, ukf->sigma()));

//         // integrate delayed measurement, currently ignoring the difference in uncertainty at the delayed state
//         ukf->update(xy_position.mu, boost::bind(measurementDelayedXYPosition<WState>, _1),
//                     boost::bind(ukfom::id<XY_Position::Cov>, xy_position.cov),
//                     d2p95<State::scalar>);

//         WState new_state = ukf->mu();
//         new_state.delayed_position = current_delayed_position;

//         ukf.reset(new MTK_UKF(new_state, ukf->sigma()));

//         return true;
//     }
//     return false;
// }

void SimplePoseUKF::resetFilterWithExternalPose(const Eigen::Affine3d &imu_in_nwu)
{
  State new_state = ukf->mu();
  new_state.position = TranslationType(imu_in_nwu.translation());
  new_state.orientation = RotationType(MTK::SO3<double>(imu_in_nwu.rotation()));
  ukf.reset(new MTK_UKF(new_state, ukf->sigma())); // elaborate whether sigma can / has to be reset or changed or if it can stay like this
}

SimplePoseUKF::RotationRate::Mu SimplePoseUKF::getRotationRate()
{
  double latitude, longitude;
  projection->navToWorld(ukf->mu().position.x(), ukf->mu().position.y(), latitude, longitude);
  Eigen::Vector3d earth_rotation = Eigen::Vector3d(pose_estimation::EARTHW * cos(latitude), 0., pose_estimation::EARTHW * sin(latitude));
  return rotation_rate - ukf->mu().bias_gyro - ukf->mu().orientation.inverse() * earth_rotation;
}
