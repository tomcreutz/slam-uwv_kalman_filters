rock_library(uwv_kalman_filters
    SOURCES VelocityUKF.cpp
    HEADERS VelocityUKF.hpp VelocityState.hpp
    DEPS_PKGCONFIG pose_estimation uwv_dynamic_model eigen3 base-types base-lib
    DEPS_CMAKE LAPACK)