#include "VelocityUKF.hpp"

using namespace uwv_filters;

const std::string VelocityUKF::acceleration_measurement = "acceleration";
const std::string VelocityUKF::thruster_rpm_measurement = "thruster_rpm_commands";
const std::string VelocityUKF::angular_velocity_measurement = "angular_velocity";

/**
 * Applies the current acceleration to update the velocity.
 */
template <typename VelocityState>
VelocityState
processModel(const VelocityState &state, const Eigen::Vector3d& acc, double delta_time)
{
    VelocityState new_state(state);
    new_state.boxplus(acc, delta_time);
    return new_state;
}

VelocityUKF::VelocityUKF(const AbstractFilter::FilterState& initial_state)
{
    setInitialState(initial_state);

    process_noise_cov = MTK_UKF::cov::Zero();
    MTK::setDiagonal(process_noise_cov, &WState::velocity, 0.0001);
}

void VelocityUKF::setupMotionModel(const underwaterVehicle::Parameters& parameters)
{
    motion_model.reset(new underwaterVehicle::DynamicModel(parameters.ctrl_order, parameters.samplingtime, parameters.sim_per_cycle));
    motion_model->initParameters(parameters);
    FilterState current_state;
    if(getCurrentState(current_state))
        motion_model->setLinearVelocity(current_state.mu.block(0,0,3,1));
}

void VelocityUKF::predictionStep(const double delta)
{
    MTK_UKF::cov process_noise = process_noise_cov;
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();

    // TODO use velocity of the model instead of accelerations

    // use motion model to determine the current acceleration
    std::map<std::string, pose_estimation::Measurement>::const_iterator thruster_command = latest_measurements.find(thruster_rpm_measurement);
    if(thruster_command != latest_measurements.end())
    {
        base::samples::Joints joints;
        joints.time = thruster_command->second.time;
        joints.elements.resize(thruster_command->second.mu.rows());
        for(unsigned i = 0; i < thruster_command->second.mu.rows(); i++)
        {
            joints.elements[i].speed = (float)thruster_command->second.mu(i);
        }
        // apply motion commands
        Eigen::Vector3d acc_pre = motion_model->getAcceleration();
        motion_model->setSamplingTime(delta);
        if(motion_model->sendRPMCommands(joints))
        {
            Eigen::Vector3d acc_post = motion_model->getAcceleration();
            acc = 0.5 * (acc_pre + acc_post);
            if(base::isnotnan(acc))
            {
                ukf->predict(boost::bind(processModel<WState>, _1, acc, delta), MTK_UKF::cov(delta * process_noise));
                return;
            }
            else
                LOG_ERROR_S << "Model predicted acceleration contains NaN values!";
        }
        else
            LOG_ERROR_S << "Failed to apply thruster commands to motion model!";
    }

    // get acceleration from measurement
    std::map<std::string, pose_estimation::Measurement>::const_iterator it = latest_measurements.find(acceleration_measurement);
    if(it != latest_measurements.end())
    {
        acc = it->second.mu;
        process_noise= it->second.cov;
    }

    ukf->predict(boost::bind(processModel<WState>, _1, acc, delta), MTK_UKF::cov(delta * process_noise));
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
    else if(measurement.measurement_name == thruster_rpm_measurement)
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
            motion_model->setAngularVelocity(measurement.mu);
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
    mu.block(3,0,3,1) = angular_velocity;
}