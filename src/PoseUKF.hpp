#ifndef _UWV_KALMAN_FILTERS_POSE_UKF_HPP_
#define _UWV_KALMAN_FILTERS_POSE_UKF_HPP_

#include <pose_estimation/UnscentedKalmanFilter.hpp>
#include <pose_estimation/Measurement.hpp>
#include <uwv_dynamic_model/DataTypes.hpp>
#include "PoseState.hpp"
#include "PoseUKFConfig.hpp"

namespace uwv_dynamic_model
{
    class DynamicModel;
}

namespace pose_estimation
{
    class GeographicProjection;
    template<typename State> class DelayedStates;
}

namespace uwv_kalman_filters
{

/**
 * This implements a full model aided inertial localization solution for autonomous underwater vehicles.
 *
 * As minimal input the filter relays on rotation rates and accelerations from an IMU and velocities from a DVL.
 * Given force and torque measurements an AUV motion model aids the velocity estimate during DVL drop outs.
 * ADCP measurements further aid the estimation in cases of DVL bottom-lock loss.
 * Given gyroscopes capable of sensing the rotation of the earth (e.g. a fiber optic gyro)
 * this filter is able to estimate it's true heading.
 *
 * NOTE: In this filter the IMU frame is, in order to keep a certain algorithmic simplicity,
 * not considered to be rotated with respect to the body frame.
 * Rotation rates and acceleration, as well as the corresponding configuration parameters
 * therefore would need to be rotated to the body frame before integrating them in this filter.
 * Due to the used grographic projection the navigation frame is in NWU (North-West-Up).
 */
class PoseUKF : public pose_estimation::UnscentedKalmanFilter<PoseState>
{
public:
    /**
     * Persistent parameters of the filter
     */
    struct PoseUKFParameter
    {
        /* IMU with respect to body frame of the robot. Might be zero */
        Eigen::Vector3d imu_in_body;
        /* Gyro bias offset in static regimen (initial bias value) (rad/s) */
        Eigen::Vector3d gyro_bias_offset;
        /* Tau value to limit the gyro bias gain in seconds */
        double gyro_bias_tau;
        /* Acceleration bias offset in static regimen (initial bias value) (m/s^s) */
        Eigen::Vector3d acc_bias_offset;
        /* Tau value to limit the acceleration bias gain in seconds */
        double acc_bias_tau;
        /* Tau value to limit the motion model inertia gain in seconds */
        double inertia_tau;
        /* Tau value to limit the motion model linear damping gain in seconds */
        double lin_damping_tau;
        /* Tau value to limit the motion model quadratic damping gain in seconds */
        double quad_damping_tau;
        /* Tau value to limit the water currents gain in seconds */
        double water_velocity_tau;
        /* Long term 1 sigma bounds for currents in m/s */
        double water_velocity_limits;
        /* Spatial scale for water current change in m/s / m */
        double water_velocity_scale;
        /* Tau value to limit the ADCP bias gain in seconds */
        double adcp_bias_tau;
        /* atmospheric pressure in pascal (N/m²) */
        double atmospheric_pressure;
        /* Tau value to limit the water density gain in seconds */
        double water_density_tau;
    };

    /* Measurements of the filter */
    MEASUREMENT(GeographicPosition, 2)
    MEASUREMENT(XY_Position, 2)
    MEASUREMENT(Z_Position, 1)
    MEASUREMENT(Pressure, 1)
    MEASUREMENT(RotationRate, 3)
    MEASUREMENT(Acceleration, 3)
    MEASUREMENT(Velocity, 3)
    MEASUREMENT(BodyEffortsMeasurement, 6)
    MEASUREMENT(WaterVelocityMeasurement, 2)
    MEASUREMENT(VisualFeatureMeasurement, 2)

public:
    /**
     * Initializes the filter from a given initial pose (IMU in NWU-navigation frame) and pose uncertainty.
     * The rest of the state and state covariance is computed from the parameters in PoseUKFConfig.
     * @param pose_filter_config filter specific configuration
     * @param model_parameters motion model parameters
     * @param imu_in_body IMU with respect to body frame of the robot. 
     *                    If no value is given it is assumed to be equal. (optional)
     */
    PoseUKF(const Eigen::Vector3d& imu_in_nwu_pos, const Eigen::Matrix3d& imu_in_nwu_pos_cov,
            const Eigen::Quaterniond& imu_in_nwu_rot, const Eigen::Matrix3d& imu_in_nwu_rot_cov,
            const PoseUKFConfig& pose_filter_config, const uwv_dynamic_model::UWVParameters& model_parameters,
            const Eigen::Affine3d& imu_in_body = Eigen::Affine3d::Identity());
    
    /**
     * Initializes the filter from a given initial state and state uncertainty.
     * @param initial_state full filter state
     * @param state_cov initial state uncertainty
     * @param location geographic location (this is used as the reference frame of geographic projection)
     * @param model_parameters motion model parameters
     * @param filter_parameter persistent parametes of the filter
     */
    PoseUKF(const State& initial_state, const Covariance& state_cov,
            const LocationConfiguration& location, const uwv_dynamic_model::UWVParameters& model_parameters,
            const PoseUKFParameter& filter_parameter);
    
    virtual ~PoseUKF() {}
    
    /**
     * Sets the process noise covariance from the filter specific configuration.
     * @param pose_filter_config filter specific configuration
     * @param imu_delta_t delta time between IMU measurements in seconds (e.g. 0.01 @ 100Hz)
     * @param imu_in_body IMU with respect to body frame of the robot. 
     *                    If no value is given it is assumed to be equal. (optional)
     */
    void setProcessNoiseFromConfig(const PoseUKFConfig& pose_filter_config, double imu_delta_t, 
                                   const Eigen::Quaterniond& imu_in_body = Eigen::Quaterniond::Identity());
    
    /**
     * Sets up the state buffer allowing to integrate delayed state measurements.
     * @param maximum_delay of measurements in seconds
     */
    void setupDelayedStateBuffer(double maximum_delay);

    /* Latitude and Longitude in WGS 84 in radian.
     * Uncertainty expressed in m on earth surface */
    void integrateMeasurement(const GeographicPosition& geo_position,
                              const Eigen::Vector3d& gps_in_body = Eigen::Vector3d::Zero());

    /* 2D Position expressed in the NWU-navigation frame */
    void integrateMeasurement(const XY_Position& xy_position);

    /* Altitude of IMU expressed in the NWU-navigation frame */
    void integrateMeasurement(const Z_Position& z_position);

    /* Pressure in liquid in pascal (N/m²) */
    void integrateMeasurement(const Pressure& pressure,
                              const Eigen::Vector3d& pressure_sensor_in_imu = Eigen::Vector3d::Zero());

    /* Rotation rates of IMU expressed in the IMU frame */
    void integrateMeasurement(const RotationRate& rotation_rate);

    /* Accelerations of IMU expressed in the IMU frame */
    void integrateMeasurement(const Acceleration& acceleration);

    /* Velocities expressed in the IMU frame */
    void integrateMeasurement(const Velocity& velocity);

    /* Forces and torques in the body frame */
    void integrateMeasurement(const BodyEffortsMeasurement& body_efforts, bool only_affect_velocity = false);
   
    /* Water Velocities from ADCP expressed in the IMU frame */
    void integrateMeasurement(const WaterVelocityMeasurement& adcp_measurements, double cell_weighting);

    /**
     * The features (usually the four corners) of a visual marker in undistorted image coordinates.
     * |marker_features| and |feature_positions| musst be of equal size and order.
     * @param marker_features image coordinates of the features in the undistorted image.
     * @param feature_positions are the positions of the featues in the marker frame.
     */
    void integrateMeasurement(const std::vector< VisualFeatureMeasurement> &marker_features,
                              const std::vector<Eigen::Vector3d>& feature_positions,
                              const Eigen::Affine3d& marker_pose, const Eigen::Matrix<double,6,6> cov_marker_pose,
                              const CameraConfiguration& camera_config, const Eigen::Affine3d& camera_in_IMU);
    
    /** Delayed 2D Position expressed in the NWU-navigation frame
     * NOTE: Requires that `setupDelayedStateBuffer` is called before.
     * @param delay delay of the measurement in seconds
     * @returns true if measurement could be integrated.
     *          Fails if the delay exceeds the maximum delay.
     */
    bool integrateDelayedMeasurement(const XY_Position& xy_position, double delay);

    /* Returns rotation rate in IMU frame */
    RotationRate::Mu getRotationRate();

protected:
    void predictionStepImpl(double delta_t);


    boost::shared_ptr<uwv_dynamic_model::DynamicModel> dynamic_model;
    boost::shared_ptr<pose_estimation::GeographicProjection> projection;
    boost::shared_ptr<pose_estimation::DelayedStates<Translation2DType>> delayed_states;
    RotationRate::Mu rotation_rate;
    PoseUKFParameter filter_parameter;
    InertiaType::vectorized_type inertia_offset;
    LinDampingType::vectorized_type lin_damping_offset;
    QuadDampingType::vectorized_type quad_damping_offset;
    double water_density_offset;
    /* Microseconds since the initialization of the filter */
    int64_t filter_ts;
};

}

#endif
