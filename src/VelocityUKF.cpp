#include "VelocityUKF.hpp"
#include <uwv_dynamic_model/ModelSimulation.hpp>

using namespace uwv_kalman_filters;

template <typename VelocityState>
VelocityState
processMotionModel(const VelocityState &state, boost::shared_ptr<uwv_dynamic_model::ModelSimulation> motion_model,
                   const Eigen::Quaterniond& orientation, const base::Vector3d& angular_velocity,
                   const base::Vector6d& body_efforts, double delta_time)
{
    // reset current state
    uwv_dynamic_model::PoseVelocityState model_state;
    model_state.position = base::Vector3d::Zero();
    model_state.orientation = orientation;
    model_state.linear_velocity = state.velocity;
    model_state.angular_velocity = angular_velocity;
    motion_model->setSamplingTime(delta_time);

    // apply joint commands
    uwv_dynamic_model::PoseVelocityState new_model_state = motion_model->sendEffort(body_efforts, model_state);

    // apply velocity delta
    Eigen::Vector3d velocity_delta = new_model_state.linear_velocity - state.velocity;
    VelocityState new_state(state);
    new_state.velocity.boxplus(velocity_delta);

    // apply velocity in z
    Eigen::Matrix<double, 1, 1> z_vel;
    z_vel(0) = (orientation * new_state.velocity).z();
    new_state.z_position.boxplus(z_vel, delta_time);
    return new_state;
}

template <typename VelocityState>
VelocityType
measurementDVL(const VelocityState &state)
{
    return state.velocity;
}

template <typename VelocityState>
ZPosType
measurementPressureSensor(const VelocityState &state)
{
    return state.z_position;
}

VelocityUKF::VelocityUKF(const State& initial_state, const Covariance& state_cov)
{
    initializeFilter(initial_state, state_cov);

    body_efforts.mu = Eigen::Matrix<double, 6, 1>::Zero();
    process_noise_cov = Covariance::Zero();
    MTK::setDiagonal(process_noise_cov, &WState::velocity, 0.0001);
}

bool VelocityUKF::setupMotionModel(const uwv_dynamic_model::UWVParameters& parameters)
{
    motion_model.reset(new uwv_dynamic_model::ModelSimulation(uwv_dynamic_model::DYNAMIC, 0.01, 1));
    motion_model->setUWVParameters(parameters);
    prediction_model.reset(new uwv_dynamic_model::ModelSimulation(uwv_dynamic_model::DYNAMIC, 0.01, 1));
    prediction_model->setUWVParameters(parameters);

    uwv_dynamic_model::PoseVelocityState model_state;
    model_state.position = base::Vector3d::Zero();
    model_state.orientation = base::Quaterniond::Identity();
    model_state.angular_velocity = angular_velocity.mu;
    State current_state;
    if(getCurrentState(current_state))
    {
        model_state.linear_velocity = current_state.velocity;
    }
    motion_model->setPose(model_state);

    return true;
}

void VelocityUKF::integrateMeasurement(const DVLMeasurement& measurement)
{
    checkMeasurment(measurement.mu, measurement.cov);
    ukf->update(measurement.mu, boost::bind(measurementDVL<State>, _1),
                boost::bind(ukfom::id< DVLMeasurement::Cov >, measurement.cov),
                ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void VelocityUKF::integrateMeasurement(const GyroMeasurement& measurement)
{
    checkMeasurment(measurement.mu, measurement.cov);
    if(motion_model)
    {
        // update angular velocity in model
        uwv_dynamic_model::PoseVelocityState model_state = motion_model->getPose();
        model_state.angular_velocity = measurement.mu;
        motion_model->setPose(model_state);
    }
    angular_velocity = measurement;
}

void VelocityUKF::integrateMeasurement(const BodyEffortsMeasurement& measurement)
{
    checkMeasurment(measurement.mu, measurement.cov);
    body_efforts = measurement;
}

void VelocityUKF::integrateMeasurement(const PressureMeasurement& measurement)
{
    checkMeasurment(measurement.mu, measurement.cov);
    ukf->update(measurement.mu, boost::bind(measurementPressureSensor<State>, _1),
                boost::bind(ukfom::id< PressureMeasurement::Cov >, measurement.cov),
                ukfom::accept_any_mahalanobis_distance<State::scalar>);
}

void VelocityUKF::predictionStepImpl(double delta)
{
    // use motion model to determine the current acceleration
    if (motion_model.get() == NULL || prediction_model.get() == NULL)
        throw std::runtime_error("Motion model is not initialized!");

    // apply motion commands
    uwv_dynamic_model::PoseVelocityState model_state = motion_model->getPose();
    ukf->predict(boost::bind(processMotionModel<WState>, _1, prediction_model, model_state.orientation,
                                angular_velocity.mu, body_efforts.mu, delta), MTK_UKF::cov(delta * process_noise_cov));

    // this motion model is updated to have a guess about the current orientation
    motion_model->setSamplingTime(delta);
    motion_model->sendEffort(body_efforts.mu);

    return;
}
