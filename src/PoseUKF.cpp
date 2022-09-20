#include "PoseUKF.hpp"
#include <math.h>
#include <uwv_dynamic_model/DynamicModel.hpp>
#include <pose_estimation/GravitationalModel.hpp>
#include <pose_estimation/GeographicProjection.hpp>
#include <pose_estimation/DelayedStates.hpp>
#include <mtk/types/S2.hpp>

using namespace uwv_kalman_filters;

// process model
template <typename FilterState>
FilterState
processModel(const FilterState &state, const Eigen::Vector3d &rotation_rate,
             boost::shared_ptr<pose_estimation::GeographicProjection> projection,
             const InertiaType::vectorized_type &inertia_offset,
             const LinDampingType::vectorized_type &lin_damping_offset,
             const QuadDampingType::vectorized_type &quad_damping_offset,
             const double water_density_offset,
             const PoseUKF::PoseUKFParameter &filter_parameter, double delta_time)
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

    InertiaType::vectorized_type inertia_delta = (-1.0 / filter_parameter.inertia_tau) *
                                                 (Eigen::Map<const InertiaType::vectorized_type>(state.inertia.data()) - inertia_offset);
    new_state.inertia.boxplus(inertia_delta, delta_time);

    LinDampingType::vectorized_type lin_damping_delta = (-1.0 / filter_parameter.lin_damping_tau) *
                                                        (Eigen::Map<const LinDampingType::vectorized_type>(state.lin_damping.data()) - lin_damping_offset);
    new_state.lin_damping.boxplus(lin_damping_delta, delta_time);

    QuadDampingType::vectorized_type quad_damping_delta = (-1.0 / filter_parameter.quad_damping_tau) *
                                                          (Eigen::Map<const QuadDampingType::vectorized_type>(state.quad_damping.data()) - quad_damping_offset);
    new_state.quad_damping.boxplus(quad_damping_delta, delta_time);

    // XY water velocity state changes due to position change over a period of time (delta P ~ V * dt). This should be reflected in the process noise.
    // Does not account for revisitation. XY water velocity also changes to due to a temporal aspect, which is also reflected here.

    // water velocity delta = (-1/water_velocity_tau) * (water velocity state) for first order markov process limits
    // if dv_dp = 1 sigma change in water velocity with distance (e.g. 0.1m/s / 100 m), then total change uncertainty = dv_dp * v * dt
    // water velocity delta covariance = time based covariance + position change based covariance

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

    // clone delayed position state
    new_state.delayed_position = Translation2DType(state.position.block(0, 0, 2, 1));

    return new_state;
}

// measurement models
template <typename FilterState>
Eigen::Matrix<TranslationType::scalar, 2, 1>
measurementXYPosition(const FilterState &state)
{
    return state.position.block(0, 0, 2, 1);
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

template <typename FilterState>
Eigen::Matrix<TranslationType::scalar, 6, 1>
measurementEfforts(const FilterState &state, boost::shared_ptr<uwv_dynamic_model::DynamicModel> dynamic_model,
                   const Eigen::Vector3d &imu_in_body, const Eigen::Vector3d &rotation_rate_body)
{
    // set damping parameters
    uwv_dynamic_model::UWVParameters params = dynamic_model->getUWVParameters();
    params.inertia_matrix.block(0, 0, 2, 2) = state.inertia.block(0, 0, 2, 2);
    params.inertia_matrix.block(0, 5, 2, 1) = state.inertia.block(0, 2, 2, 1);
    params.inertia_matrix.block(5, 0, 1, 2) = state.inertia.block(2, 0, 1, 2);
    params.inertia_matrix.block(5, 5, 1, 1) = state.inertia.block(2, 2, 1, 1);
    params.damping_matrices[0].block(0, 0, 2, 2) = state.lin_damping.block(0, 0, 2, 2);
    params.damping_matrices[0].block(0, 5, 2, 1) = state.lin_damping.block(0, 2, 2, 1);
    params.damping_matrices[0].block(5, 0, 1, 2) = state.lin_damping.block(2, 0, 1, 2);
    params.damping_matrices[0].block(5, 5, 1, 1) = state.lin_damping.block(2, 2, 1, 1);
    params.damping_matrices[1].block(0, 0, 2, 2) = state.quad_damping.block(0, 0, 2, 2);
    params.damping_matrices[1].block(0, 5, 2, 1) = state.quad_damping.block(0, 2, 2, 1);
    params.damping_matrices[1].block(5, 0, 1, 2) = state.quad_damping.block(2, 0, 1, 2);
    params.damping_matrices[1].block(5, 5, 1, 1) = state.quad_damping.block(2, 2, 1, 1);

    dynamic_model->setUWVParameters(params);

    // assume center of rotation to be the body frame
    Eigen::Vector3d water_velocity;
    water_velocity[0] = state.water_velocity[0];
    water_velocity[1] = state.water_velocity[1];
    water_velocity[2] = 0; // start with the assumption of zero water current velocity in the Z

    Eigen::Vector3d velocity_body = state.orientation.inverse() * (state.velocity) - rotation_rate_body.cross(imu_in_body);
    velocity_body = velocity_body - state.orientation.inverse() * water_velocity;
    Eigen::Matrix<TranslationType::scalar, 6, 1> velocity_6d;
    velocity_6d << velocity_body, rotation_rate_body;

    // assume center of rotation to be the body frame
    Eigen::Vector3d acceleration_body = state.orientation.inverse() * state.acceleration - rotation_rate_body.cross(rotation_rate_body.cross(imu_in_body));
    Eigen::Matrix<TranslationType::scalar, 6, 1> acceleration_6d;
    // assume the angular acceleration to be zero
    acceleration_6d << acceleration_body, Eigen::Vector3d::Zero();

    Eigen::Matrix<TranslationType::scalar, 6, 1> efforts = dynamic_model->calcEfforts(acceleration_6d, velocity_6d, state.orientation);

    // returns the expected forces and torques given the current state
    return efforts;
}

/* This measurement model allows to constrain the velocity based on the motion model in the absence of effort measurements */
template <typename FilterState>
Eigen::Matrix<TranslationType::scalar, 6, 1>
constrainVelocity(const FilterState &state, boost::shared_ptr<uwv_dynamic_model::DynamicModel> dynamic_model,
                  const Eigen::Vector3d &imu_in_body, const Eigen::Vector3d &rotation_rate_body,
                  const Eigen::Vector3d &water_velocity, const Eigen::Quaterniond &orientation,
                  const Eigen::Vector3d &acceleration_body)
{
    Eigen::Vector3d velocity_body = orientation.inverse() * (state.velocity) - rotation_rate_body.cross(imu_in_body);
    velocity_body -= orientation.inverse() * water_velocity;
    Eigen::Matrix<TranslationType::scalar, 6, 1> velocity_6d;
    velocity_6d << velocity_body, rotation_rate_body;

    Eigen::Matrix<TranslationType::scalar, 6, 1> acceleration_6d;
    // assume the angular acceleration to be zero
    acceleration_6d << acceleration_body, base::Vector3d::Zero();

    Eigen::Matrix<TranslationType::scalar, 6, 1> efforts = dynamic_model->calcEfforts(acceleration_6d, velocity_6d, orientation);

    // returns the expected forces and torques given the current state
    return efforts;
}

// delayed position update model
template <typename FilterState>
Translation2DType
measurementDelayedXYPosition(const FilterState &state)
{
    return state.delayed_position;
}

/**
 * Augments the pose filter state with a marker pose.
 * This allows to take the uncertainty of the marker pose into account.
 */
MTK_BUILD_MANIFOLD(PoseStateWithMarker,
                   ((ukfom::mtkwrap<PoseState>, filter_state))((TranslationType, marker_position)) // position of a marker in navigation frame
                   ((RotationType, marker_orientation))                                            // orientation of a marker in navigation frame
)
typedef ukfom::mtkwrap<PoseStateWithMarker> WPoseStateWithMarker;
typedef Eigen::Matrix<PoseStateWithMarker::scalar, PoseStateWithMarker::DOF, PoseStateWithMarker::DOF> PoseStateWithMarkerCov;
typedef ukfom::mtkwrap<MTK::S2<double>> WS2Type;

template <typename FilterState>
WS2Type
measurementVisualLandmark(const FilterState &state, const Eigen::Vector3d &feature_pos, const Eigen::Affine3d &cam_in_imu)
{
    Eigen::Affine3d imu_in_nav; // = Eigen::Affine3d(state.filter_state.orientation);
    imu_in_nav.translation() = state.filter_state.position;
    imu_in_nav.linear() = state.filter_state.orientation.toRotationMatrix();
    std::cout << "translation :" << imu_in_nav.translation() << "rotation : " << imu_in_nav.linear() << std::endl;
    ;
    Eigen::Affine3d nav_in_cam = (imu_in_nav * cam_in_imu).inverse();
    std::cout << "translation :" << nav_in_cam.translation() << "rotation : " << nav_in_cam.linear() << std::endl;
    ;

    Eigen::Vector3d feature_in_cam = nav_in_cam * (state.marker_orientation * feature_pos + state.marker_position);
    std::cout << "feature_in_cam vector: " << feature_in_cam << std::endl;

    return WS2Type(MTK::S2<double>(feature_in_cam));
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

PoseUKF::PoseUKF(const Eigen::Vector3d &imu_in_nwu_pos, const Eigen::Matrix3d &imu_in_nwu_pos_cov,
                 const Eigen::Quaterniond &imu_in_nwu_rot, const Eigen::Matrix3d &imu_in_nwu_rot_cov,
                 const PoseUKFConfig &filter_config, const uwv_dynamic_model::UWVParameters &model_parameters,
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
    initial_state.inertia.block(0, 0, 2, 2) = model_parameters.inertia_matrix.block(0, 0, 2, 2);
    initial_state.inertia.block(0, 2, 2, 1) = model_parameters.inertia_matrix.block(0, 5, 2, 1);
    initial_state.inertia.block(2, 0, 1, 2) = model_parameters.inertia_matrix.block(5, 0, 1, 2);
    initial_state.inertia.block(2, 2, 1, 1) = model_parameters.inertia_matrix.block(5, 5, 1, 1);
    initial_state.lin_damping.block(0, 0, 2, 2) = model_parameters.damping_matrices[0].block(0, 0, 2, 2);
    initial_state.lin_damping.block(0, 2, 2, 1) = model_parameters.damping_matrices[0].block(0, 5, 2, 1);
    initial_state.lin_damping.block(2, 0, 1, 2) = model_parameters.damping_matrices[0].block(5, 0, 1, 2);
    initial_state.lin_damping.block(2, 2, 1, 1) = model_parameters.damping_matrices[0].block(5, 5, 1, 1);
    initial_state.quad_damping.block(0, 0, 2, 2) = model_parameters.damping_matrices[1].block(0, 0, 2, 2);
    initial_state.quad_damping.block(0, 2, 2, 1) = model_parameters.damping_matrices[1].block(0, 5, 2, 1);
    initial_state.quad_damping.block(2, 0, 1, 2) = model_parameters.damping_matrices[1].block(5, 0, 1, 2);
    initial_state.quad_damping.block(2, 2, 1, 1) = model_parameters.damping_matrices[1].block(5, 5, 1, 1);
    initial_state.water_velocity = WaterVelocityType(Eigen::Vector2d::Zero());
    initial_state.water_velocity_below = WaterVelocityType(Eigen::Vector2d::Zero());
    initial_state.bias_adcp = WaterVelocityType(Eigen::Vector2d::Zero());
    Eigen::Matrix<double, 1, 1> water_density;
    water_density << filter_config.hydrostatics.water_density;
    initial_state.water_density = DensityType(water_density);
    initial_state.delayed_position = Translation2DType(imu_in_nwu_pos.head<2>());

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
    MTK::subblock(initial_state_cov, &State::inertia) = filter_config.model_noise_parameters.inertia_instability.cwiseAbs2().asDiagonal();
    MTK::subblock(initial_state_cov, &State::lin_damping) = filter_config.model_noise_parameters.lin_damping_instability.cwiseAbs2().asDiagonal();
    MTK::subblock(initial_state_cov, &State::quad_damping) = filter_config.model_noise_parameters.quad_damping_instability.cwiseAbs2().asDiagonal();
    MTK::subblock(initial_state_cov, &State::water_velocity) = pow(filter_config.water_velocity.limits, 2) * Eigen::Matrix2d::Identity();
    MTK::subblock(initial_state_cov, &State::water_velocity_below) = pow(filter_config.water_velocity.limits, 2) * Eigen::Matrix2d::Identity();
    MTK::subblock(initial_state_cov, &State::bias_adcp) = pow(filter_config.water_velocity.adcp_bias_limits, 2) * Eigen::Matrix2d::Identity();
    Eigen::Matrix<double, 1, 1> water_density_var;
    water_density_var << pow(filter_config.hydrostatics.water_density_limits, 2.);
    MTK::subblock(initial_state_cov, &State::water_density) = water_density_var;
    MTK::subblock(initial_state_cov, &State::delayed_position) = MTK::subblock(initial_state_cov, &State::position).block(0, 0, 2, 2);

    initializeFilter(initial_state, initial_state_cov);

    inertia_offset = Eigen::Map<const InertiaType::vectorized_type>(initial_state.inertia.data());
    lin_damping_offset = Eigen::Map<const LinDampingType::vectorized_type>(initial_state.lin_damping.data());
    quad_damping_offset = Eigen::Map<const QuadDampingType::vectorized_type>(initial_state.quad_damping.data());
    water_density_offset = initial_state.water_density(0);

    rotation_rate = RotationRate::Mu::Zero();

    dynamic_model.reset(new uwv_dynamic_model::DynamicModel());
    dynamic_model->setUWVParameters(model_parameters);

    projection.reset(new pose_estimation::GeographicProjection(filter_config.location.latitude, filter_config.location.longitude));

    filter_parameter.imu_in_body = imu_in_body.translation();
    filter_parameter.acc_bias_tau = filter_config.acceleration.bias_tau;
    filter_parameter.acc_bias_offset = imu_in_body.rotation() * filter_config.acceleration.bias_offset;
    filter_parameter.gyro_bias_tau = filter_config.rotation_rate.bias_tau;
    filter_parameter.gyro_bias_offset = imu_in_body.rotation() * filter_config.rotation_rate.bias_offset;
    filter_parameter.inertia_tau = filter_config.model_noise_parameters.inertia_tau;
    filter_parameter.lin_damping_tau = filter_config.model_noise_parameters.lin_damping_tau;
    filter_parameter.quad_damping_tau = filter_config.model_noise_parameters.quad_damping_tau;
    filter_parameter.water_velocity_tau = filter_config.water_velocity.tau;
    filter_parameter.water_velocity_limits = filter_config.water_velocity.limits;
    filter_parameter.water_velocity_scale = filter_config.water_velocity.scale;
    filter_parameter.adcp_bias_tau = filter_config.water_velocity.adcp_bias_tau;
    filter_parameter.atmospheric_pressure = filter_config.hydrostatics.atmospheric_pressure;
    filter_parameter.water_density_tau = filter_config.hydrostatics.water_density_tau;
}

PoseUKF::PoseUKF(const State &initial_state, const Covariance &state_cov,
                 const LocationConfiguration &location, const uwv_dynamic_model::UWVParameters &model_parameters,
                 const PoseUKFParameter &filter_parameter) : filter_parameter(filter_parameter), filter_ts(0)
{
    initializeFilter(initial_state, state_cov);

    rotation_rate = RotationRate::Mu::Zero();

    dynamic_model.reset(new uwv_dynamic_model::DynamicModel());
    dynamic_model->setUWVParameters(model_parameters);

    inertia_offset = Eigen::Map<const InertiaType::vectorized_type>(initial_state.inertia.data());
    lin_damping_offset = Eigen::Map<const LinDampingType::vectorized_type>(initial_state.lin_damping.data());
    quad_damping_offset = Eigen::Map<const QuadDampingType::vectorized_type>(initial_state.quad_damping.data());
    water_density_offset = initial_state.water_density(0);

    projection.reset(new pose_estimation::GeographicProjection(location.latitude, location.longitude));
}

void PoseUKF::setProcessNoiseFromConfig(const PoseUKFConfig &filter_config, double imu_delta_t,
                                        const Eigen::Quaterniond &imu_in_body)
{
    Eigen::Matrix3d imu_in_body_m = imu_in_body.toRotationMatrix();

    Covariance process_noise_cov = PoseUKF::Covariance::Zero();
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
    MTK::subblock(process_noise_cov, &State::inertia) = (2. / (filter_config.model_noise_parameters.inertia_tau * imu_delta_t)) *
                                                        filter_config.model_noise_parameters.inertia_instability.cwiseAbs2().asDiagonal();
    MTK::subblock(process_noise_cov, &State::lin_damping) = (2. / (filter_config.model_noise_parameters.lin_damping_tau * imu_delta_t)) *
                                                            filter_config.model_noise_parameters.lin_damping_instability.cwiseAbs2().asDiagonal();
    MTK::subblock(process_noise_cov, &State::quad_damping) = (2. / (filter_config.model_noise_parameters.quad_damping_tau * imu_delta_t)) *
                                                             filter_config.model_noise_parameters.quad_damping_instability.cwiseAbs2().asDiagonal();

    MTK::subblock(process_noise_cov, &State::water_velocity) = (2. / (filter_config.water_velocity.tau * imu_delta_t)) *
                                                               pow(filter_config.water_velocity.limits, 2) * Eigen::Matrix2d::Identity();

    MTK::subblock(process_noise_cov, &State::water_velocity_below) = (2. / (filter_config.water_velocity.tau * imu_delta_t)) *
                                                                     pow(filter_config.water_velocity.limits, 2) * Eigen::Matrix2d::Identity();

    MTK::subblock(process_noise_cov, &State::bias_adcp) = (2. / (filter_config.water_velocity.adcp_bias_tau * imu_delta_t)) *
                                                          pow(filter_config.water_velocity.adcp_bias_limits, 2) * Eigen::Matrix2d::Identity();

    Eigen::Matrix<double, 1, 1> water_density_noise;
    water_density_noise << (2. / (filter_config.hydrostatics.water_density_tau * imu_delta_t)) * pow(filter_config.hydrostatics.water_density_limits, 2.);
    MTK::subblock(process_noise_cov, &State::water_density) = water_density_noise;
    MTK::subblock(process_noise_cov, &State::delayed_position) = MTK::subblock(process_noise_cov, &State::position).block(0, 0, 2, 2);

    setProcessNoiseCovariance(process_noise_cov);
}

void PoseUKF::setupDelayedStateBuffer(double maximum_delay)
{
    delayed_states.reset(new pose_estimation::DelayedStates<Translation2DType>(std::abs(maximum_delay)));
}

void PoseUKF::predictionStepImpl(double delta_t)
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

    ukf->predict(boost::bind(processModel<WState>, _1, rotation_rate, projection,
                             inertia_offset, lin_damping_offset, quad_damping_offset, water_density_offset,
                             filter_parameter, delta_t),
                 MTK_UKF::cov(process_noise));

    // save current state
    if (delayed_states)
    {
        filter_ts += pose_estimation::DelayedStates<Translation2DType>::fromSeconds(delta_t);
        auto sigma = ukf->sigma(); // has to be copied as MTK::subblock doesnt accept const
        delayed_states->pushState(filter_ts, ukf->mu().delayed_position, MTK::subblock(sigma, &State::delayed_position));
    }
}

void PoseUKF::integrateMeasurement(const Velocity &velocity)
{
    checkMeasurment(velocity.mu, velocity.cov);
    ukf->update(velocity.mu, boost::bind(measurementVelocity<State>, _1),
                boost::bind(ukfom::id<Velocity::Cov>, velocity.cov),
                ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void PoseUKF::integrateMeasurement(const Acceleration &acceleration)
{
    checkMeasurment(acceleration.mu, acceleration.cov);
    ukf->update(acceleration.mu, boost::bind(measurementAcceleration<State>, _1),
                boost::bind(ukfom::id<Acceleration::Cov>, acceleration.cov),
                ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void PoseUKF::integrateMeasurement(const RotationRate &rotation_rate)
{
    checkMeasurment(rotation_rate.mu, rotation_rate.cov);
    this->rotation_rate = rotation_rate.mu;
}

void PoseUKF::integrateMeasurement(const Z_Position &z_position)
{
    checkMeasurment(z_position.mu, z_position.cov);
    ukf->update(z_position.mu, boost::bind(measurementZPosition<State>, _1),
                boost::bind(ukfom::id<Z_Position::Cov>, z_position.cov),
                ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void PoseUKF::integrateMeasurement(const XY_Position &xy_position)
{
    checkMeasurment(xy_position.mu, xy_position.cov);
    ukf->update(xy_position.mu, boost::bind(measurementXYPosition<State>, _1),
                boost::bind(ukfom::id<XY_Position::Cov>, xy_position.cov),
                ukfom::accept_any_mahalanobis_distance<State::scalar>); // d2p95<State::scalar>);
}

void PoseUKF::integrateMeasurement(const Pressure &pressure, const Eigen::Vector3d &pressure_sensor_in_imu)
{
    checkMeasurment(pressure.mu, pressure.cov);
    ukf->update(pressure.mu, boost::bind(measurementPressureSensor<State>, _1, pressure_sensor_in_imu, filter_parameter.atmospheric_pressure),
                boost::bind(ukfom::id<Pressure::Cov>, pressure.cov),
                ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void PoseUKF::integrateMeasurement(const GeographicPosition &geo_position, const Eigen::Vector3d &gps_in_body)
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

void PoseUKF::integrateMeasurement(const BodyEffortsMeasurement &body_efforts, bool only_affect_velocity)
{
    checkMeasurment(body_efforts.mu, body_efforts.cov);

    if (only_affect_velocity)
    {
        // this allows to contstrain only the velocity using the motion model
        Eigen::Vector3d water_velocity(ukf->mu().water_velocity.x(), ukf->mu().water_velocity.y(), 0.);
        Eigen::Vector3d rotation_rate_body = getRotationRate();
        // assume center of rotation to be the body frame
        Eigen::Vector3d acceleration_body = ukf->mu().orientation.inverse() * ukf->mu().acceleration - rotation_rate_body.cross(rotation_rate_body.cross(filter_parameter.imu_in_body));
        ukf->update(body_efforts.mu, boost::bind(constrainVelocity<State>, _1, dynamic_model, filter_parameter.imu_in_body, rotation_rate_body, water_velocity, ukf->mu().orientation, acceleration_body),
                    boost::bind(ukfom::id<BodyEffortsMeasurement::Cov>, body_efforts.cov),
                    ukfom::accept_any_mahalanobis_distance<State::scalar>);
    }
    else
    {
        ukf->update(body_efforts.mu, boost::bind(measurementEfforts<State>, _1, dynamic_model, filter_parameter.imu_in_body, getRotationRate()),
                    boost::bind(ukfom::id<BodyEffortsMeasurement::Cov>, body_efforts.cov),
                    ukfom::accept_any_mahalanobis_distance<State::scalar>);
    }
}

void PoseUKF::integrateMeasurement(const WaterVelocityMeasurement &adcp_measurements, double cell_weighting)
{
    checkMeasurment(adcp_measurements.mu, adcp_measurements.cov);

    ukf->update(adcp_measurements.mu, boost::bind(measurementWaterCurrents<State>, _1, cell_weighting),
                boost::bind(ukfom::id<WaterVelocityMeasurement::Cov>, adcp_measurements.cov),
                d2p95<State::scalar>);
}

void PoseUKF::integrateMeasurement(const std::vector<VisualFeatureMeasurement> &marker_corners,
                                   const std::vector<Eigen::Vector3d> &feature_positions,
                                   const Eigen::Affine3d &marker_pose, const Eigen::Matrix<double, 6, 6> cov_marker_pose,
                                   const CameraConfiguration &camera_config, const Eigen::Affine3d &camera_in_IMU)
{
    // Augment the filter state with the marker pose
    WPoseStateWithMarker augmented_state;
    augmented_state.filter_state = ukf->mu();
    augmented_state.marker_position = TranslationType(marker_pose.translation());
    augmented_state.marker_orientation = RotationType(MTK::SO3<double>(marker_pose.rotation()));
    PoseStateWithMarkerCov augmented_state_cov = PoseStateWithMarkerCov::Zero();
    augmented_state_cov.block(0, 0, WState::DOF, WState::DOF) = ukf->sigma();
    augmented_state_cov.bottomRightCorner<6, 6>() = cov_marker_pose;
    ukfom::ukf<WPoseStateWithMarker> augmented_ukf(augmented_state, augmented_state_cov);

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
        Eigen::Matrix2d projection_cov;
        projection_cov << marker_corners[i].cov(0, 0) / fx2, marker_corners[i].cov(0, 1) / fxy,
            marker_corners[i].cov(1, 0) / fxy, marker_corners[i].cov(1, 1) / fy2;

        augmented_ukf.update(projection, boost::bind(measurementVisualLandmark<WPoseStateWithMarker>, _1, feature_positions[i], camera_in_IMU),
                             boost::bind(ukfom::id<VisualFeatureMeasurement::Cov>, projection_cov),
                             ukfom::accept_any_mahalanobis_distance<WPoseStateWithMarker::scalar>);
    }

    // Reconstructing the filter is currently the only way to modify the internal state of the filter
    ukf.reset(new MTK_UKF(augmented_ukf.mu().filter_state, augmented_ukf.sigma().block(0, 0, WState::DOF, WState::DOF)));
}

bool PoseUKF::integrateDelayedMeasurement(const XY_Position &xy_position, double delay)
{
    checkMeasurment(xy_position.mu, xy_position.cov);
    int64_t measurement_ts = filter_ts - pose_estimation::DelayedStates<Translation2DType>::fromSeconds(delay);
    Translation2DType delayed_position;
    pose_estimation::DelayedStates<Translation2DType>::Cov cov_delayed_position;
    if (delayed_states && delayed_states->getClosestState(measurement_ts, delayed_position, cov_delayed_position))
    {
        // update current state with closest delayed state
        Translation2DType current_delayed_position = ukf->mu().delayed_position;
        WState delayed_state = ukf->mu();
        delayed_state.delayed_position = delayed_position;
        ukf.reset(new MTK_UKF(delayed_state, ukf->sigma()));

        // integrate delayed measurement, currently ignoring the difference in uncertainty at the delayed state
        ukf->update(xy_position.mu, boost::bind(measurementDelayedXYPosition<WState>, _1),
                    boost::bind(ukfom::id<XY_Position::Cov>, xy_position.cov),
                    d2p95<State::scalar>);

        WState new_state = ukf->mu();
        new_state.delayed_position = current_delayed_position;

        ukf.reset(new MTK_UKF(new_state, ukf->sigma()));

        return true;
    }
    return false;
}

PoseUKF::RotationRate::Mu PoseUKF::getRotationRate()
{
    double latitude, longitude;
    projection->navToWorld(ukf->mu().position.x(), ukf->mu().position.y(), latitude, longitude);
    Eigen::Vector3d earth_rotation = Eigen::Vector3d(pose_estimation::EARTHW * cos(latitude), 0., pose_estimation::EARTHW * sin(latitude));
    return rotation_rate - ukf->mu().bias_gyro - ukf->mu().orientation.inverse() * earth_rotation;
}
