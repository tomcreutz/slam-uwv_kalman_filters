#pragma once

#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/UKF.hpp>
#include <uwv_dynamic_model/ModelSimulation.hpp>
#include "VelocityState.hpp"
#include <map>
#include <boost/shared_ptr.hpp>

namespace uwv_kalman_filters
{

class VelocityUKF : public pose_estimation::UKF<pose_estimation::VelocityState>
{
public:
    static const std::string acceleration_measurement;
    static const std::string body_efforts_measurement;
    static const std::string angular_velocity_measurement;

    VelocityUKF(const FilterState& initial_state);
    virtual ~VelocityUKF() {}

    bool setupMotionModel(const uwv_dynamic_model::UWVParameters& parameters);

    void predictionStep(const double delta);

    bool getCurrentState(FilterState& current_state);

protected:
    void correctionStepUser(const pose_estimation::Measurement& measurement);

    void muToUKFState(const FilterState::Mu &mu, WState& state) const;
    void UKFStateToMu(const WState& state, FilterState::Mu &mu) const;

protected:
    std::map<std::string, pose_estimation::Measurement> latest_measurements;
    boost::shared_ptr<uwv_dynamic_model::ModelSimulation> motion_model;
    boost::shared_ptr<uwv_dynamic_model::ModelSimulation> prediction_model;
};

}