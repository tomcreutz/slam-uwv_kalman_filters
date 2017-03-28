rock_library(uwv_kalman_filters
    SOURCES VelocityUKF.cpp PoseUKF.cpp
    HEADERS VelocityUKF.hpp PoseUKF.hpp PoseState.hpp PoseUKFConfig.hpp
    DEPS_PKGCONFIG pose_estimation uwv_dynamic_model eigen3 base-types base-lib base-logging
    DEPS_CMAKE LAPACK)