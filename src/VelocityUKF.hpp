#pragma once

#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/UKF.hpp>
#include <uwv_dynamic_model/uwv_dynamic_model.hpp>
#include "VelocityState.hpp"
#include <map>
#include <boost/shared_ptr.hpp>

namespace uwv_filters
{

class VelocityUKF : public pose_estimation::UKF<pose_estimation::VelocityState>
{
public:
    static const std::string acceleration_measurement;
    static const std::string thruster_rpm_measurement;
    static const std::string angular_velocity_measurement;

    VelocityUKF(const FilterState& initial_state);
    virtual ~VelocityUKF() {}

    void setupMotionModel(const underwaterVehicle::Parameters& parameters);

    void predictionStep(const double delta);

    bool getCurrentState(FilterState& current_state);

protected:
    void correctionStepUser(const pose_estimation::Measurement& measurement);

    void muToUKFState(const FilterState::Mu &mu, WState& state) const;
    void UKFStateToMu(const WState& state, FilterState::Mu &mu) const;

protected:
    std::map<std::string, pose_estimation::Measurement> latest_measurements;
    boost::shared_ptr<underwaterVehicle::DynamicModel> motion_model;
};

}