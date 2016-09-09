#pragma once

#include <pose_estimation/UnscentedKalmanFilter.hpp>
#include <pose_estimation/Measurement.hpp>

#include <uwv_dynamic_model/DataTypes.hpp>
#include <boost/shared_ptr.hpp>
#include <mtk/types/SOn.hpp>
#include <mtk/types/vect.hpp>
#include <mtk/startIdx.hpp>
#include <mtk/build_manifold.hpp>

namespace uwv_dynamic_model
{
    class ModelSimulation;
}

namespace uwv_kalman_filters
{

typedef ukfom::mtkwrap< MTK::vect<3, double> > VelocityType;
typedef ukfom::mtkwrap< MTK::vect<1, double> > ZPosType;

MTK_BUILD_MANIFOLD(VelocityState,
   ((VelocityType, velocity))
   ((ZPosType, z_position))
)

class VelocityUKF : public pose_estimation::UnscentedKalmanFilter<VelocityState>
{
public:
    MEASUREMENT(DVLMeasurement, 3)
    MEASUREMENT(GyroMeasurement, 3)
    MEASUREMENT(BodyEffortsMeasurement, 6)
    MEASUREMENT(PressureMeasurement, 1)

public:
    VelocityUKF(const State& initial_state, const Covariance& state_cov);
    virtual ~VelocityUKF() {}

    bool setupMotionModel(const uwv_dynamic_model::UWVParameters& parameters);

    void integrateMeasurement(const DVLMeasurement& measurement);
    void integrateMeasurement(const GyroMeasurement& measurement);
    void integrateMeasurement(const BodyEffortsMeasurement& measurement);
    void integrateMeasurement(const PressureMeasurement& measurement);

protected:
    void predictionStepImpl(double delta_t);

protected:
    boost::shared_ptr<uwv_dynamic_model::ModelSimulation> motion_model;
    boost::shared_ptr<uwv_dynamic_model::ModelSimulation> prediction_model;
    GyroMeasurement angular_velocity;
    BodyEffortsMeasurement body_efforts;
};

}