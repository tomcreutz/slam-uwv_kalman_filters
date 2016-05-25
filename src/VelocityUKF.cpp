#include "VelocityUKF.hpp"

using namespace uwv_kalman_filters;

const std::string VelocityUKF::acceleration_measurement = "acceleration";
const std::string VelocityUKF::body_efforts_measurement = "body_efforts";
const std::string VelocityUKF::angular_velocity_measurement = "angular_velocity";

/**
 * Applies the current acceleration to update the velocity.
 */
template <typename VelocityState>
VelocityState
processAcc(const VelocityState &state, const Eigen::Vector3d& acc, double delta_time)
{
    VelocityState new_state(state);
    new_state.velocity.boxplus(acc, delta_time);

    Eigen::Matrix<double,1,1> z_vel;
    z_vel(0) = new_state.velocity.z();
    new_state.z_position.boxplus(z_vel, delta_time);
    return new_state;
}

template <typename VelocityState>
VelocityState
processMotionModel(const VelocityState &state, uwv_dynamic_model::ModelSimulation& motion_model,
                   const Eigen::Quaterniond& orientation, const base::Vector3d& angular_velocity,
                   const base::Vector6d& body_efforts, double delta_time)
{
    // reset current state
    uwv_dynamic_model::PoseVelocityState model_state;
    model_state.position = base::Vector3d::Zero();
    model_state.orientation = orientation;
    model_state.linear_velocity = state.velocity;
    model_state.angular_velocity = angular_velocity;
    motion_model.setSamplingTime(delta_time);

    // apply joint commands
    motion_model.sendEffort(body_efforts, model_state);

    // apply velocity delta
    uwv_dynamic_model::PoseVelocityState new_model_state = motion_model.getPose();
    Eigen::Vector3d velocity_delta = new_model_state.linear_velocity - state.velocity;
    VelocityState new_state(state);
    new_state.velocity.boxplus(velocity_delta);

    // apply velocity in z
    Eigen::Matrix<double, 1, 1> z_vel;
    z_vel(0) = (orientation * new_state.velocity).z();
    new_state.z_position.boxplus(z_vel, delta_time);
    return new_state;
}

VelocityUKF::VelocityUKF(const AbstractFilter::FilterState& initial_state)
{
    setInitialState(initial_state);

    process_noise_cov = MTK_UKF::cov::Zero();
    MTK::setDiagonal(process_noise_cov, &WState::velocity, 0.0001);
}

bool VelocityUKF::setupMotionModel(const uwv_dynamic_model::UWVParameters& parameters)
{
    motion_model.reset(new uwv_dynamic_model::ModelSimulation(uwv_dynamic_model::DYNAMIC, 0.01, 1));
    motion_model->setUWVParameters(parameters);
    prediction_model.reset(new uwv_dynamic_model::ModelSimulation(uwv_dynamic_model::DYNAMIC, 0.01, 1));
    prediction_model->setUWVParameters(parameters);

    FilterState current_state;
    if(getCurrentState(current_state))
    {
        uwv_dynamic_model::PoseVelocityState model_state;
        model_state.position = base::Vector3d::Zero();
        model_state.orientation = base::Quaterniond::Identity();
        model_state.linear_velocity = current_state.mu.block(0,0,3,1);
        model_state.angular_velocity = current_state.mu.block(3,0,3,1);
        motion_model->setPose(model_state);
    }
    return true;
}

void VelocityUKF::predictionStep(const double delta)
{
    MTK_UKF::cov process_noise = process_noise_cov;

    // use motion model to determine the current acceleration
    std::map<std::string, pose_estimation::Measurement>::const_iterator body_efforts = latest_measurements.find(body_efforts_measurement);
    if(body_efforts != latest_measurements.end())
    {
        if (motion_model.get() == NULL || prediction_model.get() == NULL)
            throw std::runtime_error("Motion model is not initialized!");

        // apply motion commands
        uwv_dynamic_model::PoseVelocityState model_state = motion_model->getPose();
        ukf->predict(boost::bind(processMotionModel<WState>, _1, *prediction_model, model_state.orientation,
                                 model_state.angular_velocity, body_efforts->second.mu, delta), MTK_UKF::cov(delta * process_noise));

        // this motion model is updated to have a guess about the current orientation
        motion_model->setSamplingTime(delta);
        motion_model->sendEffort(body_efforts->second.mu);

        return;
    }

    // get acceleration from measurement
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    std::map<std::string, pose_estimation::Measurement>::const_iterator it = latest_measurements.find(acceleration_measurement);
    if(it != latest_measurements.end())
    {
        acc = it->second.mu;
        process_noise= it->second.cov;
    }

    ukf->predict(boost::bind(processAcc<WState>, _1, acc, delta), MTK_UKF::cov(delta * process_noise));
}

bool VelocityUKF::getCurrentState(FilterState& current_state)
{
    if(ukf.get() != NULL)
    {
        UKFStateToMu(ukf->mu(), current_state.mu);

        current_state.cov.resize(WState::DOF + 3, WState::DOF + 3);
        current_state.cov.setZero();
        current_state.cov.block(0,0,WState::DOF,WState::DOF) = ukf->sigma();

        // augment covariance with latest angular velocity covariance
        Eigen::Matrix3d angular_velocity_cov = Eigen::Matrix3d::Zero();
        std::map<std::string, pose_estimation::Measurement>::const_iterator it = latest_measurements.find(angular_velocity_measurement);
        if(it != latest_measurements.end())
        {
            angular_velocity_cov = it->second.cov;
        }
        current_state.cov.block(WState::DOF,WState::DOF,3,3) = angular_velocity_cov;
        return true;
    }
    return false;
}

void VelocityUKF::correctionStepUser(const pose_estimation::Measurement& measurement)
{
    if(measurement.measurement_name == acceleration_measurement)
    {
        latest_measurements[measurement.measurement_name] = measurement;
    }
    else if(measurement.measurement_name == body_efforts_measurement)
    {
        if(motion_model)
            latest_measurements[measurement.measurement_name] = measurement;
        else
            LOG_ERROR_S << "Cannot handle thruster commands, since motion model is not initialized!";
    }
    else if(measurement.measurement_name == angular_velocity_measurement)
    {
        if(motion_model)
        {
            // update angular velocity in model
            uwv_dynamic_model::PoseVelocityState model_state = motion_model->getPose();
            model_state.angular_velocity = measurement.mu;
            motion_model->setPose(model_state);
        }
        latest_measurements[measurement.measurement_name] = measurement;
    }
    else
        LOG_ERROR_S << "Measurement " << measurement.measurement_name << " is not supported by the VelocityUKF filter.";
}

void VelocityUKF::muToUKFState(const FilterState::Mu& mu, WState& state) const
{
    assert(mu.rows() >= WState::DOF);

    state.velocity = mu.block(0,0,3,1);
    state.z_position = mu.block(3,0,1,1);
}

void VelocityUKF::UKFStateToMu(const WState& state, FilterState::Mu& mu) const
{
    mu.resize(WState::DOF + 3);
    mu.setZero();

    // augment state with latest angular velocity
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    std::map<std::string, pose_estimation::Measurement>::const_iterator it = latest_measurements.find(angular_velocity_measurement);
    if(it != latest_measurements.end())
    {
        angular_velocity = it->second.mu;
    }

    mu.block(0,0,3,1) = state.velocity;
    mu.block(3,0,1,1) = state.z_position;
    mu.block(4,0,3,1) = angular_velocity;
}
