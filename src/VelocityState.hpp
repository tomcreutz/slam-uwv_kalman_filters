#ifndef _POSE_ESTIMATION_VELOCITY_STATE_HPP_
#define _POSE_ESTIMATION_VELOCITY_STATE_HPP_

/** MTK library **/
#include <mtk/src/SubManifold.hpp>
#include <mtk/types/SOn.hpp>
#include <mtk/types/vect.hpp>
#include <mtk/build_manifold.hpp>
#include <mtk/startIdx.hpp>
#include <pose_estimation/ManifoldHelper.hpp>

namespace pose_estimation
{

typedef MTK::vect<3, double> VelocityType;

MTK_BUILD_MANIFOLD(VelocityState,
   ((VelocityType, velocity))
)

template<>
inline void getStateVector(const VelocityState &state, Eigen::Matrix<VelocityState::scalar, VelocityState::DOF, 1>& state_vector)
{
    state_vector.block(MTK::getStartIdx(&VelocityState::velocity),0,MTK::getDof(&VelocityState::velocity),1) = state.velocity;
}

}

#endif