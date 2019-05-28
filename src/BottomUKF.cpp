#include "BottomUKF.hpp"

using namespace uwv_kalman_filters;

template <typename BottomState>
BottomState
processModel(const BottomState &state, double velocity, double delta_time)
{
    BottomState new_state(state);

    double velocity_inv = -1. * velocity;
    Eigen::Map< const Eigen::Matrix<double, 1, 1> > velocity_inv_map = Eigen::Map< const Eigen::Matrix<double, 1, 1> >(&velocity_inv);
    new_state.distance.boxplus(velocity_inv_map, delta_time);

    return new_state;
}

template <typename BottomState>
DistanceType
measurementDistance(const BottomState &state, const Eigen::Vector3d& unit_direction, const Eigen::Vector3d& origin)
{
    Eigen::Vector3d bottom(0., 0., -state.distance.value);
    Eigen::Vector3d normal = state.normal.get_vect();

    double v = unit_direction.dot(normal);
    if(v != 0.)
        return DistanceType((bottom - origin).dot(normal) / v);

    return DistanceType(0.);
}

template <typename BottomState>
NormalType
measurementNormal(const BottomState &state)
{
    return state.normal;
}


BottomUKF::BottomUKF(const BottomState& initial_state, const pose_estimation::UnscentedKalmanFilter< BottomState >::Covariance& state_cov)
    : velocity(Eigen::Vector3d::Zero())
{
    initializeFilter(initial_state, state_cov);

    process_noise_cov = Covariance::Identity();
}

void BottomUKF::predictionStepImpl(double delta_t)
{
    MTK_UKF::cov process_noise_cov_scaled = std::pow(velocity.head<2>().norm(), 2.) * pow(delta_t, 2.) * process_noise_cov;

    ukf->predict(boost::bind(processModel<WState>, _1, velocity.z(), delta_t),
                 process_noise_cov_scaled);
}

void BottomUKF::integrateMeasurement(const RangeMeasurement& measurement, const Eigen::Vector3d& unit_direction, const Eigen::Vector3d& origin)
{
    checkMeasurment(measurement.mu, measurement.cov);
    ukf->update(DistanceType(measurement.mu(0)), boost::bind(measurementDistance<WState>, _1, unit_direction, origin),
                boost::bind(ukfom::id< RangeMeasurement::Cov >, measurement.cov));
}

void BottomUKF::integrateMeasurement(const NormalType& measurement, const Eigen::Matrix2d& measurement_cov)
{
    ukf->update(measurement, boost::bind(measurementNormal<WState>, _1),
                boost::bind(ukfom::id< Eigen::Matrix2d >, measurement_cov));
}

void BottomUKF::setVelocity(const Eigen::Vector3d& velocity)
{
    this->velocity = velocity;
}