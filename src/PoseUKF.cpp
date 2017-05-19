#include "PoseUKF.hpp"
#include <math.h>
#include <base/Float.hpp>
#include <base-logging/Logging.hpp>
#include <uwv_dynamic_model/DynamicModel.hpp>
#include <pose_estimation/GravitationalModel.hpp>
#include <pose_estimation/GeographicProjection.hpp>

using namespace uwv_kalman_filters;

// process model
template <typename FilterState>
FilterState
processModel(const FilterState &state, const Eigen::Vector3d& rotation_rate,
             boost::shared_ptr<pose_estimation::GeographicProjection> projection,
             const LinDampingType::vectorized_type& lin_damping_offset,
             const QuadDampingType::vectorized_type& quad_damping_offset,
             const PoseUKF::PoseUKFParameter& filter_parameter, double delta_time)
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

    Eigen::Vector3d gyro_bias_delta = (-1.0/filter_parameter.gyro_bias_tau) * state.bias_gyro;
    new_state.bias_gyro.boxplus(gyro_bias_delta, delta_time);

    Eigen::Vector3d acc_bias_delta = (-1.0/filter_parameter.acc_bias_tau) * state.bias_acc;
    new_state.bias_acc.boxplus(acc_bias_delta, delta_time);

    LinDampingType::vectorized_type lin_damping_delta = (-1.0/filter_parameter.lin_damping_tau) *
                        (Eigen::Map< const LinDampingType::vectorized_type >(state.lin_damping.data()) - lin_damping_offset);
    new_state.lin_damping.boxplus(lin_damping_delta, delta_time);

    QuadDampingType::vectorized_type quad_damping_delta = (-1.0/filter_parameter.quad_damping_tau) *
                        (Eigen::Map< const QuadDampingType::vectorized_type >(state.quad_damping.data()) - quad_damping_offset);
    new_state.quad_damping.boxplus(quad_damping_delta, delta_time);
    return new_state;
}

// measurement models
template <typename FilterState>
Eigen::Matrix<TranslationType::scalar, 2, 1>
measurementXYPosition(const FilterState &state)
{
    return state.position.block(0,0,2,1);
}

template <typename FilterState>
Eigen::Matrix<TranslationType::scalar, 1, 1>
measurementZPosition(const FilterState &state)
{
    return state.position.block(2,0,1,1);
}

template <typename FilterState>
VelocityType
measurementVelocityConstantOrientation(const FilterState &state, const Eigen::Quaterniond& orientation)
{
    // return expected velocities in the IMU frame
    return VelocityType(orientation.inverse() * state.velocity);
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
Eigen::Matrix<TranslationType::scalar, 3, 1>
measurementEfforts(const FilterState &state, boost::shared_ptr<uwv_dynamic_model::DynamicModel> dynamic_model,
                   const Eigen::Vector3d& imu_in_body, const Eigen::Vector3d& rotation_rate_body, const Eigen::Quaterniond& orientation)
{
    // set damping parameters
    uwv_dynamic_model::UWVParameters params = dynamic_model->getUWVParameters();
    params.damping_matrices[0].block(0,0,2,2) = state.lin_damping.block(0,0,2,2).cwiseAbs();
    params.damping_matrices[0].block(0,5,2,1) = state.lin_damping.block(0,2,2,1).cwiseAbs();
    params.damping_matrices[1].block(0,0,2,2) = state.quad_damping.block(0,0,2,2).cwiseAbs();
    params.damping_matrices[1].block(0,5,2,1) = state.quad_damping.block(0,2,2,1).cwiseAbs();
    dynamic_model->setUWVParameters(params);

    // assume center of rotation to be the body frame
    Eigen::Vector3d velocity_body = orientation.inverse() * state.velocity - rotation_rate_body.cross(imu_in_body);
    base::Vector6d velocity_6d;
    velocity_6d << velocity_body, rotation_rate_body;

    // assume center of rotation to be the body frame
    Eigen::Vector3d acceleration_body = orientation.inverse() * state.acceleration - rotation_rate_body.cross(rotation_rate_body.cross(imu_in_body));
    base::Vector6d acceleration_6d;
    // assume the angular acceleration to be zero
    acceleration_6d << acceleration_body, base::Vector3d::Zero();

    base::Vector6d efforts = dynamic_model->calcEfforts(acceleration_6d, velocity_6d, orientation);

    // returns the expected linear body efforts given the current state
    return efforts.head<3>();
}

PoseUKF::PoseUKF(const State& initial_state, const Covariance& state_cov,
                const LocationConfiguration& location, const uwv_dynamic_model::UWVParameters& model_parameters,
                const PoseUKFParameter& filter_parameter) : filter_parameter(filter_parameter)
{
    initializeFilter(initial_state, state_cov);

    rotation_rate = RotationRate::Mu::Zero();

    dynamic_model.reset(new uwv_dynamic_model::DynamicModel());
    dynamic_model->setUWVParameters(model_parameters);

    lin_damping_offset = Eigen::Map< const LinDampingType::vectorized_type >(initial_state.lin_damping.data());
    quad_damping_offset = Eigen::Map< const QuadDampingType::vectorized_type >(initial_state.quad_damping.data());

    projection.reset(new pose_estimation::GeographicProjection(location.latitude, location.longitude));
}


void PoseUKF::predictionStepImpl(double delta_t)
{
    Eigen::Matrix3d rot = ukf->mu().orientation.matrix();
    Covariance process_noise = process_noise_cov;
    // uncertainty matrix calculations
    MTK::subblock(process_noise, &State::orientation) = rot * MTK::subblock(process_noise_cov, &State::orientation) * rot.transpose();
    process_noise = pow(delta_t, 2.) * process_noise;

    ukf->predict(boost::bind(processModel<WState>, _1, rotation_rate, projection, lin_damping_offset, quad_damping_offset, filter_parameter, delta_t),
                 MTK_UKF::cov(process_noise));
}

void PoseUKF::integrateMeasurement(const Velocity& velocity)
{
    checkMeasurment(velocity.mu, velocity.cov);

    int idx_yaw = MTK::getStartIdx(&State::orientation) + 2;
    double var_yaw = ukf->sigma()(idx_yaw, idx_yaw);
    if(sqrt(var_yaw) < filter_parameter.heading_converged_std)
    {
        ukf->update(velocity.mu, boost::bind(measurementVelocity<State>, _1),
            boost::bind(ukfom::id< Velocity::Cov >, velocity.cov),
            ukfom::accept_any_mahalanobis_distance<State::scalar>);
    }
    else
    {
        ukf->update(velocity.mu, boost::bind(measurementVelocityConstantOrientation<State>, _1, ukf->mu().orientation),
            boost::bind(ukfom::id< Velocity::Cov >, velocity.cov),
            ukfom::accept_any_mahalanobis_distance<State::scalar>);
    }
}

void PoseUKF::integrateMeasurement(const Acceleration& acceleration)
{
    checkMeasurment(acceleration.mu, acceleration.cov);
    ukf->update(acceleration.mu, boost::bind(measurementAcceleration<State>, _1),
                boost::bind(ukfom::id< Acceleration::Cov >, acceleration.cov),
                ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void PoseUKF::integrateMeasurement(const RotationRate& rotation_rate)
{
    checkMeasurment(rotation_rate.mu, rotation_rate.cov);
    this->rotation_rate = rotation_rate.mu;
}

void PoseUKF::integrateMeasurement(const Z_Position& z_position)
{
    checkMeasurment(z_position.mu, z_position.cov);
    ukf->update(z_position.mu, boost::bind(measurementZPosition<State>, _1),
                boost::bind(ukfom::id< Z_Position::Cov >, z_position.cov),
                ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void PoseUKF::integrateMeasurement(const XY_Position& xy_position)
{
    checkMeasurment(xy_position.mu, xy_position.cov);
    ukf->update(xy_position.mu, boost::bind(measurementXYPosition<State>, _1),
                boost::bind(ukfom::id< XY_Position::Cov >, xy_position.cov),
                ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void PoseUKF::integrateMeasurement(const GeographicPosition& geo_position, const Eigen::Vector3d& gps_in_body)
{
    checkMeasurment(geo_position.mu, geo_position.cov);

    // project geographic position to local NWU plane
    Eigen::Matrix<TranslationType::scalar, 2, 1> projected_position;
    projection->worldToNav(geo_position.mu.x(), geo_position.mu.y(), projected_position.x(), projected_position.y());
    projected_position = projected_position - (ukf->mu().orientation * gps_in_body).head<2>();

    ukf->update(projected_position, boost::bind(measurementXYPosition<State>, _1),
                boost::bind(ukfom::id< XY_Position::Cov >, geo_position.cov),
                ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void PoseUKF::integrateMeasurement(const BodyEffortsMeasurement& body_efforts)
{
    checkMeasurment(body_efforts.mu, body_efforts.cov);

    ukf->update(body_efforts.mu, boost::bind(measurementEfforts<State>, _1, dynamic_model, filter_parameter.imu_in_body, getRotationRate(), ukf->mu().orientation),
                boost::bind(ukfom::id< BodyEffortsMeasurement::Cov >, body_efforts.cov),
                ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

PoseUKF::RotationRate::Mu PoseUKF::getRotationRate()
{
    double latitude, longitude;
    projection->navToWorld(ukf->mu().position.x(), ukf->mu().position.y(), latitude, longitude);
    Eigen::Vector3d earth_rotation = Eigen::Vector3d(pose_estimation::EARTHW * cos(latitude), 0., pose_estimation::EARTHW * sin(latitude));
    return rotation_rate - ukf->mu().bias_gyro - ukf->mu().orientation.inverse() * earth_rotation;
}