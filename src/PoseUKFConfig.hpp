#ifndef _UWV_KALMAN_FILTERS_POSE_UKF_CONFIG_HPP
#define _UWV_KALMAN_FILTERS_POSE_UKF_CONFIG_HPP

#include <base/Eigen.hpp>

namespace uwv_kalman_filters
{

struct WaterVelocityParameters
{
    /* time scale for water current change */
    double tau;

    /* limits of the water current change from temporal change */
    double limits;

    /* Standard deviation of the water velocity measurements ((m/s)/sqrt(Hz)) */
    base::Vector3d measurement_std;

    /* rate change of currents based on spatial change */
    double scale;

    /* Water current cell size in meter */
    double cell_size;

    /* Water current first cell blank in meter */
    double first_cell_blank;

    /* Minimum correltation of ADCP measurements */
    double minimum_correlation;

    /* Time scale for change of ADCP bias in seconds */
    double adcp_bias_tau;

    /* ADCP bias standard deviation */
    double adcp_bias_limits;
};

struct InertialNoiseParameters
{
    /*  Random walk ((m/s^s)/sqrt(Hz) for accelerometers or (rad/s)/sqrt(Hz) for gyros) */
    base::Vector3d randomwalk;

    /*  Bias offset in static regimen (initial bias value) */
    base::Vector3d bias_offset;

    /* Bias instability (m/s^2 for accelerometers or rad/s for gyros) */
    base::Vector3d bias_instability;

    /* Tau value to limit the bias gain in seconds */
    double bias_tau;
};

struct DynamicModelNoiseParameters
{
    /* Standard deviation of body effort measurements.
     * Forces in (N/sqrt(Hz)) and torques in (Nm/sqrt(Hz)) */
    base::Vector6d body_efforts_std;

    /* Moment of inertia instability (kg*m^2)
     * The instability is mapped to the x, yx, xy, y, xψ, yψ components of
     * the inertia matrix. ψ is the rotation around the z-axis.
     */
    base::Vector6d inertia_instability;

    /* Liniar damping parameter instability (kg/s)
     * The instability is mapped to the x, yx, xy, y, xψ, yψ components of
     * the linear damping matrix. ψ is the rotation around the z-axis.
     */
    base::Vector6d lin_damping_instability;

    /* Quadratic damping parameter instability (kg/m)
     * The instability is mapped to the x, yx, xy, y, xψ, yψ components of
     * the quadratic damping matrix. ψ is the rotation around the z-axis.
     */
    base::Vector6d quad_damping_instability;

    /* Tau value to limit the bias gain in seconds */
    double inertia_tau;

    /* Tau value to limit the bias gain in seconds */
    double lin_damping_tau;

    /* Tau value to limit the bias gain in seconds */
    double quad_damping_tau;
};

struct LocationConfiguration
{
    /* Latitude in radians */
    double latitude;

    /* Longitude in radians */
    double longitude;

    /* Altitude in meters */
    double altitude;
};

struct PoseUKFConfig
{
    /* Inerial noise parameters for acceleration */
    InertialNoiseParameters acceleration;

    /** Inerial noise parameters for acceleration */
    InertialNoiseParameters rotation_rate;

    /** Noise parameter for the dynamic motion model */
    DynamicModelNoiseParameters model_noise_parameters;

    /* Water velocity parameters */
    WaterVelocityParameters water_velocity;

    /* Latitude and Longitude of operational area */
    LocationConfiguration location;

    /** Max change of acceleration in m/s^3 */
    base::Vector3d max_jerk;

    /** Max effort in N and Nm
     * This is used to define the uncertainty of
     * unknown effort measurements.
     * E.g. if the AUV is on the surface.
     */
    base::Vector6d max_effort;

    /* Minimum depth of the AUV to apply the dynamic model  */
    double dynamic_model_min_depth;
};

}

#endif