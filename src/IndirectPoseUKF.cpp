#include "IndirectPoseUKF.hpp"
#include <mtk/types/S2.hpp>

using namespace uwv_kalman_filters;

// process model
template <typename FilterState>
FilterState
processModel(const FilterState &state, double orientation_error_tau, double delta_time)
{
    FilterState new_state(state);

    // limits the allowed error in orientation over time tau
    Eigen::Matrix<double, 3 ,1> orientation_error_delta;
    orientation_error_delta << MTK::SO3<double>::log(state.orientation_error);
    orientation_error_delta = (-1.0/orientation_error_tau) * orientation_error_delta;
    new_state.orientation_error.boxplus(orientation_error_delta, delta_time);
    
    return new_state;
}

/**
 * Augments the pose filter state with a marker pose.
 * This allows to take the uncertainty of the marker pose into account.
 */
MTK_BUILD_MANIFOLD(FilterStateWithMarker,
   ((ukfom::mtkwrap<PoseErrorState>, filter_state))
   ((TranslationType, marker_position)) // position of a marker in navigation frame
   ((RotationType, marker_orientation)) // orientation of a marker in navigation frame
)
typedef ukfom::mtkwrap<FilterStateWithMarker> WFilterStateWithMarker;
typedef Eigen::Matrix<FilterStateWithMarker::scalar, FilterStateWithMarker::DOF, FilterStateWithMarker::DOF> FilterStateWithMarkerCov;
typedef ukfom::mtkwrap< MTK::S2<double> > WS2Type;

/**
 * Computes the angles in S2 to a feature with known pose given the current system state
 */
template <typename FilterState>
WS2Type
measurementVisualLandmark(const FilterState &state, const Eigen::Vector3d& feature_pos, const Eigen::Affine3d& cam_in_body, const Eigen::Affine3d& body_in_nav)
{
    Eigen::Affine3d pose_error = Eigen::Affine3d(state.filter_state.orientation_error);
    pose_error.translation() = state.filter_state.position_error;
    
    Eigen::Affine3d nav_in_cam = ((body_in_nav * pose_error) * cam_in_body).inverse();
    
    Eigen::Vector3d feature_in_cam = nav_in_cam * (state.marker_orientation * feature_pos + state.marker_position);
    
    return WS2Type(MTK::S2<double>(feature_in_cam));
}


IndirectPoseUKF::IndirectPoseUKF(const Eigen::Vector3d& position_error_std, 
                                 const Eigen::Vector3d& orientation_error_std, 
                                 double orientation_error_tau,
                                 const Eigen::Vector3d& initial_position_error,
                                 const Eigen::Vector3d& initial_position_error_std) : 
    pose_ref(Eigen::Affine3d::Identity()), orientation_error_tau(orientation_error_tau)
{
    // set initial state
    State initial_state;
    initial_state.position_error = TranslationType(initial_position_error);
    initial_state.orientation_error = RotationType(Eigen::Quaterniond::Identity());

    // set initial state covariance
    Covariance initial_state_cov = Covariance::Zero();
    MTK::subblock(initial_state_cov, &State::position_error) = initial_position_error_std.cwiseAbs2().asDiagonal();
    MTK::subblock(initial_state_cov, &State::orientation_error) = orientation_error_std.cwiseAbs2().asDiagonal();
    
    initializeFilter(initial_state, initial_state_cov);
    
    // set process noise
    Covariance process_noise_cov = Covariance::Zero();
    MTK::subblock(process_noise_cov, &State::position_error) = position_error_std.cwiseAbs2().asDiagonal();
    MTK::subblock(process_noise_cov, &State::orientation_error) = orientation_error_std.cwiseAbs2().asDiagonal();
    
    setProcessNoiseCovariance(process_noise_cov);
}

void IndirectPoseUKF::predictionStepImpl(double delta_t)
{
    Eigen::Matrix3d rot = ukf->mu().orientation_error.matrix();
    Covariance process_noise = process_noise_cov;
    // uncertainty matrix calculations
    MTK::subblock(process_noise, &State::orientation_error) = rot * ((2. / (orientation_error_tau * delta_t)) * 
                                                            MTK::subblock(process_noise_cov, &State::orientation_error)) * rot.transpose();
        
    process_noise = pow(delta_t, 2.) * process_noise;
    
    ukf->predict(boost::bind(processModel<WState>, _1, orientation_error_tau, delta_t),
                 MTK_UKF::cov(process_noise));
}

void IndirectPoseUKF::integrateMeasurement(const std::vector<VisualFeatureMeasurement>& marker_features, 
                                            const std::vector<Eigen::Vector3d>& feature_positions, 
                                            const Eigen::Affine3d& marker_pose, const Eigen::Matrix<double, 6, 6> cov_marker_pose, 
                                            const uwv_kalman_filters::CameraConfiguration& camera_config, const Eigen::Affine3d& camera_in_body)
{
    // Augment the filter state with the marker pose
    WFilterStateWithMarker augmented_state;
    augmented_state.filter_state = ukf->mu();
    augmented_state.marker_position = TranslationType(marker_pose.translation());
    augmented_state.marker_orientation = RotationType(MTK::SO3<double>(marker_pose.rotation()));
    FilterStateWithMarkerCov augmented_state_cov = FilterStateWithMarkerCov::Zero();
    augmented_state_cov.block(0,0, WState::DOF, WState::DOF) = ukf->sigma();
    augmented_state_cov.bottomRightCorner<6,6>() = cov_marker_pose;
    ukfom::ukf<WFilterStateWithMarker> augmented_ukf(augmented_state, augmented_state_cov);

    double fx2 = std::pow(camera_config.fx, 2.);
    double fy2 = std::pow(camera_config.fy, 2.);
    double fxy = camera_config.fx * camera_config.fy;

    // Apply measurements on the augmented state
    for(unsigned i = 0; i < marker_features.size() || i < feature_positions.size(); i++)
    {
        checkMeasurment(marker_features[i].mu, marker_features[i].cov);

        // project image coordinates into S2
        WS2Type projection(MTK::S2<double>((marker_features[i].mu.x() - camera_config.cx) / camera_config.fx,
                                           (marker_features[i].mu.y() - camera_config.cy) / camera_config.fy,
                                            1.0));
        Eigen::Matrix2d projection_cov;
        projection_cov << marker_features[i].cov(0,0) / fx2, marker_features[i].cov(0,1) / fxy,
                          marker_features[i].cov(1,0) / fxy, marker_features[i].cov(1,1) / fy2;

        augmented_ukf.update(projection, boost::bind(measurementVisualLandmark<WFilterStateWithMarker>, _1, 
                                                     feature_positions[i], camera_in_body, pose_ref),
            boost::bind(ukfom::id< VisualFeatureMeasurement::Cov >, projection_cov),
            ukfom::accept_any_mahalanobis_distance<WFilterStateWithMarker::scalar>);

    }

    // Reconstructing the filter is currently the only way to modify the internal state of the filter
    ukf.reset(new MTK_UKF(augmented_ukf.mu().filter_state, augmented_ukf.sigma().block(0,0, WState::DOF, WState::DOF)));
}

void IndirectPoseUKF::getCorrectedPose(Eigen::Affine3d& pose_corrected) const
{
    Eigen::Affine3d pose_error = Eigen::Affine3d(ukf->mu().orientation_error);
    pose_error.translation() = ukf->mu().position_error;
    pose_corrected = pose_ref * pose_error;
}

void IndirectPoseUKF::updatePoseReference(const Eigen::Affine3d& pose_ref)
{
    this->pose_ref = pose_ref;
}
