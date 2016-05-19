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
processAcc(const VelocityState &state, const Eigen::Vector3d& acc, double delta_time)
{
    VelocityState new_state(state);
    new_state.boxplus(acc, delta_time);
    return new_state;
}

template <typename VelocityState>
VelocityState
processMotionModel(const VelocityState &state, underwaterVehicle::DynamicModel& motion_model,
                   const Eigen::Quaterniond& orientation, const base::Vector3d& angular_velocity,
                   const base::samples::Joints& joints, double delta_time)
{
    // reset current state
    motion_model.setPosition(base::Vector3d::Zero());
    motion_model.setOrientation(orientation);
    motion_model.setLinearVelocity(state.velocity);
    motion_model.setAngularVelocity(angular_velocity);
    motion_model.setSamplingTime(delta_time);

    // apply joint commands
    if (!motion_model.sendRPMCommands(joints))
        throw std::runtime_error("Failed to apply thruster commands in pretiction step!");

    // apply velocity delta
    base::Vector3d linear_velocity;
    motion_model.getLinearVelocity(linear_velocity);
    Eigen::Vector3d velocity_delta = linear_velocity - state.velocity;
    VelocityState new_state(state);
    new_state.boxplus(velocity_delta);
    return new_state;
}

VelocityUKF::VelocityUKF(const AbstractFilter::FilterState& initial_state)
{
    setInitialState(initial_state);

    process_noise_cov = MTK_UKF::cov::Zero();
    MTK::setDiagonal(process_noise_cov, &WState::velocity, 0.0001);
}

bool VelocityUKF::setupMotionModel(const underwaterVehicle::Parameters& parameters)
{
    motion_model.reset(new underwaterVehicle::DynamicModel(parameters.ctrl_order, parameters.samplingtime, parameters.sim_per_cycle));
    if (!motion_model->initParameters(parameters))
        return false;
    prediction_model.reset(new underwaterVehicle::DynamicModel(parameters.ctrl_order, parameters.samplingtime, parameters.sim_per_cycle));
    if (!prediction_model->initParameters(parameters))
        return false;
    FilterState current_state;
    if(getCurrentState(current_state))
        motion_model->setLinearVelocity(current_state.mu.block(0,0,3,1));
    return true;
}

void VelocityUKF::predictionStep(const double delta)
{
    MTK_UKF::cov process_noise = process_noise_cov;

    // use motion model to determine the current acceleration
    std::map<std::string, pose_estimation::Measurement>::const_iterator thruster_command = latest_measurements.find(thruster_rpm_measurement);
    if(thruster_command != latest_measurements.end())
    {
        if (motion_model.get() == NULL || prediction_model.get() == NULL)
            throw std::runtime_error("Motion model is not initialized!");

        base::samples::Joints joints;
        joints.time = thruster_command->second.time;
        joints.elements.resize(thruster_command->second.mu.rows());
        for(unsigned i = 0; i < thruster_command->second.mu.rows(); i++)
        {
            joints.elements[i].speed = (float)thruster_command->second.mu(i);
        }

        // apply motion commands
        base::Vector3d angular_velocity;
        motion_model->getAngularVelocity(angular_velocity);
        base::Quaterniond orientation;
        motion_model->getQuatOrienration(orientation);
        ukf->predict(boost::bind(processMotionModel<WState>, _1, *prediction_model, orientation,
                                 angular_velocity, joints, delta), MTK_UKF::cov(delta * process_noise));

        // this motion model is updated to have a guess about the current orientation
        motion_model->setSamplingTime(delta);
        if (!motion_model->sendRPMCommands(joints))
        {
            LOG_ERROR_S << "Failed to apply thruster commands to motion model!";
        }

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