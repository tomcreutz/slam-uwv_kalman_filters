#ifndef UWV_KALMAN_FILTERS_POSEUKF_POSE_STATE_HPP_
#define UWV_KALMAN_FILTERS_POSEUKF_POSE_STATE_HPP_

/** MTK library **/
#include <mtk/build_manifold.hpp>
#include <mtk/src/SubManifold.hpp>
#include <mtk/startIdx.hpp>
#include <mtk/types/SOn.hpp>
#include <mtk/types/vect.hpp>
#include <ukfom/mtkwrap.hpp>

namespace uwv_kalman_filters
{

typedef ukfom::mtkwrap<MTK::SO3<double>> RotationType;
typedef ukfom::mtkwrap<RotationType::vect_type> TranslationType;
typedef ukfom::mtkwrap<RotationType::vect_type> VelocityType;
typedef ukfom::mtkwrap<RotationType::vect_type> AccelerationType;
typedef ukfom::mtkwrap<RotationType::vect_type> BiasType;
typedef ukfom::mtkwrap<MTK::vect<1>> GravityType;
typedef ukfom::mtkwrap<MTK::vect<1>> DensityType;

MTK_BUILD_MANIFOLD(
  SimplePoseState,
  ((TranslationType, position))    // position of IMU in navigation frame
    ((RotationType, orientation)) // orientation of IMU in navigation frame
    ((VelocityType, velocity))   // velocity of IMU in navigation frame
    ((AccelerationType, acceleration)) // acceleration of IMU in navigation frame
    ((BiasType, bias_gyro)) // gyroscope bias states
    ((BiasType, bias_acc))  // acceleration bias states
//    ((GravityType, gravity)) // local gravity, to refine the WGS-84 ellipsoid earth gravity model
//    ((DensityType, water_density)) // water density in kg/m³
)

}  // namespace uwv_kalman_filters

#endif
