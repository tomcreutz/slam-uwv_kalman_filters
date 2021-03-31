rock_library(uwv_kalman_filters
    SOURCES VelocityUKF.cpp
            PoseUKF.cpp
            BottomUKF.cpp
    HEADERS VelocityUKF.hpp
            PoseUKF.hpp
            PoseState.hpp
            PoseUKFConfig.hpp
            BottomUKF.hpp
    DEPS_PKGCONFIG pose_estimation
                   uwv_dynamic_model
                   eigen3
    DEPS_CMAKE LAPACK)
