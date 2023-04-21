#ifndef _UWV_KALMAN_FILTERS_INDIRECT_POSE_UKF_HPP_
#define _UWV_KALMAN_FILTERS_INDIRECT_POSE_UKF_HPP_

#include <pose_estimation/UnscentedKalmanFilter.hpp>
#include <pose_estimation/Measurement.hpp>
#include "PoseUKFConfig.hpp"

#include <mtk/types/SOn.hpp>
#include <mtk/types/vect.hpp>
#include <mtk/startIdx.hpp>
#include <mtk/build_manifold.hpp>

namespace uwv_kalman_filters
{

typedef ukfom::mtkwrap< MTK::SO3<double> > RotationType;
typedef ukfom::mtkwrap<RotationType::vect_type> TranslationType;

MTK_BUILD_MANIFOLD(PoseErrorState,
   ((TranslationType, position_error)) // position error of body in navigation frame
   ((RotationType, orientation_error)) // orientation error of body in navigation frame
)

/**
 * Estimates the bias between a given pose reference and additional pose correcting measurements.
 * The position can have a arbitrary offset while the orientation is treated as a constraint bias.
 * Note that this will not be able to improve the uncertainty on the reference pose.
 */
class IndirectPoseUKF : public pose_estimation::UnscentedKalmanFilter<PoseErrorState>
{
public:

    MEASUREMENT(VisualFeatureMeasurement, 2)

public:
    /**
     * Initializes the filter
     * @param position_error_std Position error standard deviation
     * @param orientation_error_std Orientation error standard deviation
     * @param orientation_error_tau Time scale for orientation error change in seconds
     * @param initial_position_error Initial position error
     * @param initial_position_error_std Initial position error standard deviation
     */
    IndirectPoseUKF(const Eigen::Vector3d& position_error_std,
                    const Eigen::Vector3d& orientation_error_std,
                    double orientation_error_tau,
                    const Eigen::Vector3d& initial_position_error = Eigen::Vector3d::Zero(),
                    const Eigen::Vector3d& initial_position_error_std = Eigen::Vector3d::Ones());
    
    virtual ~IndirectPoseUKF() {}
    
    /**
     * Updates the current reference pose (body in world frame)
     */
    void updatePoseReference(const Eigen::Affine3d& pose_ref);
    
    /**
     * Returns the corrected pose, i.e. the reference pose + the estimated pose error
     */
    void getCorrectedPose(Eigen::Affine3d& pose_corrected) const;

    /**
     * The features (usually the four corners) of a visual marker in undistorted image coordinates.
     * |marker_features| and |feature_positions| must be of equal size and order.
     * @param marker_features image coordinates of the features in the undistorted image with uncertainty.
     * @param feature_positions are the positions of the features in the marker frame.
     * @param marker_pose known pose of the marker in the world frame
     * @param cov_marker_pose uncertainty of the marker pose
     * @param camera_config focal length and optical center
     * @param camera_in_body camera frame expressed in body frame
     */
    void integrateMeasurement(const std::vector< VisualFeatureMeasurement> &marker_features,
                              const std::vector<Eigen::Vector3d>& feature_positions,
                              const Eigen::Affine3d& marker_pose, const Eigen::Matrix<double,6,6> cov_marker_pose,
                              const CameraConfiguration& camera_config, const Eigen::Affine3d& camera_in_body);
    

protected:
    void predictionStepImpl(double delta_t);
    
    Eigen::Affine3d pose_ref;
    double orientation_error_tau;
};

}

#endif

