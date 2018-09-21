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
             const InertiaType::vectorized_type& inertia_offset,
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

    Eigen::Vector3d gyro_bias_delta = (-1.0/filter_parameter.gyro_bias_tau) *
                        (Eigen::Vector3d(state.bias_gyro) - filter_parameter.gyro_bias_offset);
    new_state.bias_gyro.boxplus(gyro_bias_delta, delta_time);

    Eigen::Vector3d acc_bias_delta = (-1.0/filter_parameter.acc_bias_tau) *
                        (Eigen::Vector3d(state.bias_acc) - filter_parameter.acc_bias_offset);
    new_state.bias_acc.boxplus(acc_bias_delta, delta_time);

    InertiaType::vectorized_type inertia_delta = (-1.0/filter_parameter.inertia_tau) *
                        (Eigen::Map< const InertiaType::vectorized_type >(state.inertia.data()) - inertia_offset);
    new_state.inertia.boxplus(inertia_delta, delta_time);

    LinDampingType::vectorized_type lin_damping_delta = (-1.0/filter_parameter.lin_damping_tau) *
                        (Eigen::Map< const LinDampingType::vectorized_type >(state.lin_damping.data()) - lin_damping_offset);
    new_state.lin_damping.boxplus(lin_damping_delta, delta_time);

    QuadDampingType::vectorized_type quad_damping_delta = (-1.0/filter_parameter.quad_damping_tau) *
                        (Eigen::Map< const QuadDampingType::vectorized_type >(state.quad_damping.data()) - quad_damping_offset);
    new_state.quad_damping.boxplus(quad_damping_delta, delta_time);
    
    //XY water velocity state changes due to position change over a period of time (delta P ~ V * dt). This should be reflected in the process noise. 
    //Does not account for revisitation. XY water velocity also changes to due to a temporal aspect, which is also reflected here.
    
    // water velocity delta = (-1/water_velocity_tau) * (water velocity state) for first order markov process limits
    // if dv_dp = 1 sigma change in water velocity with distance (e.g. 0.1m/s / 100 m), then total change uncertainty = dv_dp * v * dt
    // water velocity delta covariance = time based covariance + position change based covariance
    
    WaterVelocityType::vectorized_type water_velocity_delta = (-1.0/filter_parameter.water_velocity_tau) *
                        (Eigen::Map< const WaterVelocityType::vectorized_type >( state.water_velocity.data() ) ) ;
    new_state.water_velocity.boxplus(water_velocity_delta, delta_time);
    
    WaterVelocityType::vectorized_type water_velocity_below_delta = (-1.0/filter_parameter.water_velocity_tau) *
                        (Eigen::Map< const WaterVelocityType::vectorized_type >( state.water_velocity_below.data() ) ) ;
    new_state.water_velocity_below.boxplus(water_velocity_below_delta, delta_time);
    
    WaterVelocityType::vectorized_type bias_adcp_delta = (-1.0/filter_parameter.adcp_bias_tau) *
                        (Eigen::Map< const WaterVelocityType::vectorized_type >( state.bias_adcp.data() ) ) ;
    new_state.bias_adcp.boxplus(bias_adcp_delta, delta_time);
    
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
    expected_measurement[0] = cell_weighting * water_velocity_below[0] + (1-cell_weighting) * water_velocity[0] + state.bias_adcp[0];
    expected_measurement[1] = cell_weighting * water_velocity_below[1] + (1-cell_weighting) * water_velocity[1] + state.bias_adcp[1];

    return expected_measurement;
}

template <typename FilterState>
Eigen::Matrix<TranslationType::scalar, 6, 1>
measurementEfforts(const FilterState &state, boost::shared_ptr<uwv_dynamic_model::DynamicModel> dynamic_model,
                   const Eigen::Vector3d& imu_in_body, const Eigen::Vector3d& rotation_rate_body)
{
    // set damping parameters
    uwv_dynamic_model::UWVParameters params = dynamic_model->getUWVParameters();
    params.inertia_matrix.block(0,0,2,2) = state.inertia.block(0,0,2,2).cwiseAbs();
    params.inertia_matrix.block(0,5,2,1) = state.inertia.block(0,2,2,1).cwiseAbs();
    params.damping_matrices[0].block(0,0,2,2) = state.lin_damping.block(0,0,2,2).cwiseAbs();
    params.damping_matrices[0].block(0,5,2,1) = state.lin_damping.block(0,2,2,1).cwiseAbs();
    params.damping_matrices[1].block(0,0,2,2) = state.quad_damping.block(0,0,2,2).cwiseAbs();
    params.damping_matrices[1].block(0,5,2,1) = state.quad_damping.block(0,2,2,1).cwiseAbs();
       
    dynamic_model->setUWVParameters(params);

    // assume center of rotation to be the body frame
    Eigen::Vector3d water_velocity;
    water_velocity[0] = state.water_velocity[0];
    water_velocity[1] = state.water_velocity[1];
    water_velocity[2] = 0; // start with the assumption of zero water current velocity in the Z
    
    Eigen::Vector3d velocity_body = state.orientation.inverse() * (state.velocity) - rotation_rate_body.cross(imu_in_body);
    velocity_body = velocity_body - state.orientation.inverse() * water_velocity;
    base::Vector6d velocity_6d;
    velocity_6d << velocity_body, rotation_rate_body;

    // assume center of rotation to be the body frame
    Eigen::Vector3d acceleration_body = state.orientation.inverse() * state.acceleration - rotation_rate_body.cross(rotation_rate_body.cross(imu_in_body));
    base::Vector6d acceleration_6d;
    // assume the angular acceleration to be zero
    acceleration_6d << acceleration_body, base::Vector3d::Zero();

    base::Vector6d efforts = dynamic_model->calcEfforts(acceleration_6d, velocity_6d, state.orientation);

    // returns the expected forces and torques given the current state
    return efforts;
}

/* This measurement model allows to constrain the velocity based on the motion model in the absence of effort measurements */
template <typename FilterState>
Eigen::Matrix<TranslationType::scalar, 6, 1>
constrainVelocity(const FilterState &state, boost::shared_ptr<uwv_dynamic_model::DynamicModel> dynamic_model,
                   const Eigen::Vector3d& imu_in_body, const Eigen::Vector3d& rotation_rate_body,
                   const Eigen::Vector3d& water_velocity, const Eigen::Quaterniond& orientation,
                   const Eigen::Vector3d& acceleration_body)
{
    Eigen::Vector3d velocity_body = orientation.inverse() * (state.velocity) - rotation_rate_body.cross(imu_in_body);
    velocity_body -= orientation.inverse() * water_velocity;
    base::Vector6d velocity_6d;
    velocity_6d << velocity_body, rotation_rate_body;

    base::Vector6d acceleration_6d;
    // assume the angular acceleration to be zero
    acceleration_6d << acceleration_body, base::Vector3d::Zero();

    base::Vector6d efforts = dynamic_model->calcEfforts(acceleration_6d, velocity_6d, orientation);

    // returns the expected forces and torques given the current state
    return efforts;
}

//functions for innovation gate test, using mahalanobis distance
template <typename scalar_type>
static bool d2p99(const scalar_type &mahalanobis2)
{
    if(mahalanobis2>9.21) // for 2 degrees of freedom, 99% likelihood = 9.21, https://www.uam.es/personal_pdi/ciencias/anabz/Prest/Trabajos/Critical_Values_of_the_Chi-Squared_Distribution.pdf
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
    if(mahalanobis2>5.991) // for 2 degrees of freedom, 95% likelihood = 5.991, https://www.uam.es/personal_pdi/ciencias/anabz/Prest/Trabajos/Critical_Values_of_the_Chi-Squared_Distribution.pdf
    {
        return false;
    }
    else
    {
        return true;
    }
}

PoseUKF::PoseUKF(const State& initial_state, const Covariance& state_cov,
                const LocationConfiguration& location, const uwv_dynamic_model::UWVParameters& model_parameters,
                const PoseUKFParameter& filter_parameter) : filter_parameter(filter_parameter)
{
    initializeFilter(initial_state, state_cov);

    rotation_rate = RotationRate::Mu::Zero();

    dynamic_model.reset(new uwv_dynamic_model::DynamicModel());
    dynamic_model->setUWVParameters(model_parameters);

    inertia_offset = Eigen::Map< const InertiaType::vectorized_type >(initial_state.inertia.data());
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
    
    Eigen::Vector3d scaled_velocity = ukf->mu().velocity;
    scaled_velocity[2] = 10*scaled_velocity[2]; // scale Z velocity to have 10x more impact
    
    MTK::subblock(process_noise, &State::water_velocity) = MTK::subblock(process_noise_cov, &State::water_velocity) 
    + Eigen::Matrix<double,2,2>::Identity() * filter_parameter.water_velocity_scale * scaled_velocity.squaredNorm() * delta_t;
    
    MTK::subblock(process_noise, &State::water_velocity_below) = MTK::subblock(process_noise_cov, &State::water_velocity_below) 
    + Eigen::Matrix<double,2,2>::Identity() * filter_parameter.water_velocity_scale * scaled_velocity.squaredNorm() * delta_t;
        
    process_noise = pow(delta_t, 2.) * process_noise;
    
    ukf->predict(boost::bind(processModel<WState>, _1, rotation_rate, projection,
                            inertia_offset, lin_damping_offset, quad_damping_offset,
                            filter_parameter, delta_t),
                 MTK_UKF::cov(process_noise));
}

void PoseUKF::integrateMeasurement(const Velocity& velocity)
{
    checkMeasurment(velocity.mu, velocity.cov);
    ukf->update(velocity.mu, boost::bind(measurementVelocity<State>, _1),
        boost::bind(ukfom::id< Velocity::Cov >, velocity.cov),
        ukfom::accept_any_mahalanobis_distance<State::scalar>);
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
                d2p95<State::scalar>);
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
                d2p95<State::scalar>);
}

void PoseUKF::integrateMeasurement(const BodyEffortsMeasurement& body_efforts, bool only_affect_velocity)
{
    checkMeasurment(body_efforts.mu, body_efforts.cov);

    if(only_affect_velocity)
    {
        // this allows to contstrain only the velocity using the motion model
        Eigen::Vector3d water_velocity(ukf->mu().water_velocity.x(), ukf->mu().water_velocity.y(), 0.);
        Eigen::Vector3d rotation_rate_body = getRotationRate();
        // assume center of rotation to be the body frame
        Eigen::Vector3d acceleration_body = ukf->mu().orientation.inverse() * ukf->mu().acceleration - rotation_rate_body.cross(rotation_rate_body.cross(filter_parameter.imu_in_body));
        ukf->update(body_efforts.mu, boost::bind(constrainVelocity<State>, _1, dynamic_model,
                                        filter_parameter.imu_in_body, rotation_rate_body, water_velocity,
                                        ukf->mu().orientation, acceleration_body),
                    boost::bind(ukfom::id< BodyEffortsMeasurement::Cov >, body_efforts.cov),
                    ukfom::accept_any_mahalanobis_distance<State::scalar>);
    }
    else
    {
        ukf->update(body_efforts.mu, boost::bind(measurementEfforts<State>, _1, dynamic_model, filter_parameter.imu_in_body,
                                                 getRotationRate()),
                    boost::bind(ukfom::id< BodyEffortsMeasurement::Cov >, body_efforts.cov),
                    ukfom::accept_any_mahalanobis_distance<State::scalar>);
    }
}

void PoseUKF::integrateMeasurement(const WaterVelocityMeasurement& adcp_measurements, double cell_weighting)
{
    checkMeasurment(adcp_measurements.mu, adcp_measurements.cov);
    
    ukf->update(adcp_measurements.mu, boost::bind(measurementWaterCurrents<State>, _1, cell_weighting),
        boost::bind(ukfom::id< WaterVelocityMeasurement::Cov >, adcp_measurements.cov),
        d2p95<State::scalar>);
}

PoseUKF::RotationRate::Mu PoseUKF::getRotationRate()
{
    double latitude, longitude;
    projection->navToWorld(ukf->mu().position.x(), ukf->mu().position.y(), latitude, longitude);
    Eigen::Vector3d earth_rotation = Eigen::Vector3d(pose_estimation::EARTHW * cos(latitude), 0., pose_estimation::EARTHW * sin(latitude));
    return rotation_rate - ukf->mu().bias_gyro - ukf->mu().orientation.inverse() * earth_rotation;
}