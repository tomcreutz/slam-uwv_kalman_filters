#ifndef _UWV_KALMAN_FILTERS_BOTTOM_UKF_HPP_
#define _UWV_KALMAN_FILTERS_BOTTOM_UKF_HPP_

#include <pose_estimation/UnscentedKalmanFilter.hpp>
#include <pose_estimation/Measurement.hpp>

#include <mtk/types/S2.hpp>
#include <mtk/types/vect.hpp>
#include <mtk/startIdx.hpp>
#include <mtk/build_manifold.hpp>

namespace uwv_kalman_filters
{

typedef ukfom::mtkwrap< MTK::Scalar<double> > DistanceType;
typedef ukfom::mtkwrap< MTK::S2<double> > NormalType;

MTK_BUILD_MANIFOLD(BottomState,
   ((DistanceType, distance)) // positive distance in meter to the bottom surface
   ((NormalType, normal)) // normal vector of the bottom surface
)

/**
 * This filter estimates the distance and normal of the bottom surface.
 * Measurements are the vehicle velocity and the four range measurements of a DVL.
 */
class BottomUKF : public pose_estimation::UnscentedKalmanFilter<BottomState>
{
public:
    MEASUREMENT(RangeMeasurement, 1)

public:
    BottomUKF(const State& initial_state, const Covariance& state_cov);
    virtual ~BottomUKF() {}

    /** DVL based range measurements */
    void integrateMeasurement(const RangeMeasurement& measurement, 
                              const Eigen::Vector3d& unit_direction, const Eigen::Vector3d& origin);

    /** Bottom surface normal measurement (optional) */
    void integrateMeasurement(const NormalType& measurement, const Eigen::Matrix2d& measurement_cov);

    /** Sets the current velocity of the vehicle */
    void setVelocity(const Eigen::Vector3d& velocity);


protected:
    void predictionStepImpl(double delta_t);

protected:
    Eigen::Vector3d velocity;
};

}

#endif