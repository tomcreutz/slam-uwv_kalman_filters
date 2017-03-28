#ifndef _UWV_KALMAN_FILTERS_POSE_STATE_HPP_
#define _UWV_KALMAN_FILTERS_POSE_STATE_HPP_

/** MTK library **/
#include <mtk/src/SubManifold.hpp>
#include <mtk/types/SOn.hpp>
#include <mtk/types/vect.hpp>
#include <mtk/build_manifold.hpp>
#include <mtk/startIdx.hpp>
#include <ukfom/mtkwrap.hpp>

namespace uwv_kalman_filters
{

typedef ukfom::mtkwrap< MTK::SO3<double> > RotationType;
typedef ukfom::mtkwrap<RotationType::vect_type> TranslationType;
typedef ukfom::mtkwrap<RotationType::vect_type> VelocityType;
typedef ukfom::mtkwrap<RotationType::vect_type> AccelerationType;
typedef ukfom::mtkwrap<RotationType::vect_type> BiasType;
typedef ukfom::mtkwrap< MTK::vect<1> > GravityType;

MTK_BUILD_MANIFOLD(PoseState,
   ((TranslationType, position)) // position of IMU in navigation/target frame
   ((RotationType, orientation)) // orientation of IMU in navigation/target frame
   ((VelocityType, velocity)) // velocity of IMU in navigation/target frame
   ((AccelerationType, acceleration)) // acceleration in IMU frame
   ((BiasType, bias_gyro))
   ((BiasType, bias_acc))
   ((GravityType, gravity))
)

}

#endif