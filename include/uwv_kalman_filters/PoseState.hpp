#ifndef _UWV_KALMAN_FILTERS_POSE_STATE_HPP_
#define _UWV_KALMAN_FILTERS_POSE_STATE_HPP_

/** MTK library **/
#include <mtk/build_manifold.hpp>
#include <mtk/src/SubManifold.hpp>
#include <mtk/startIdx.hpp>
#include <mtk/types/SOn.hpp>
#include <mtk/types/vect.hpp>
#include <ukfom/mtkwrap.hpp>

namespace uwv_kalman_filters {

typedef ukfom::mtkwrap<MTK::SO3<double>> RotationType;
typedef ukfom::mtkwrap<RotationType::vect_type> TranslationType;
typedef ukfom::mtkwrap<RotationType::vect_type> VelocityType;
typedef ukfom::mtkwrap<RotationType::vect_type> AccelerationType;
typedef ukfom::mtkwrap<RotationType::vect_type> BiasType;
typedef ukfom::mtkwrap<MTK::vect<1>> GravityType;
typedef ukfom::mtkwrap<MTK::matrix<3, 3>> InertiaType;
typedef ukfom::mtkwrap<MTK::matrix<3, 3>> LinDampingType;
typedef ukfom::mtkwrap<MTK::matrix<3, 3>> QuadDampingType;
typedef ukfom::mtkwrap<MTK::vect<2>> WaterVelocityType;
typedef ukfom::mtkwrap<MTK::vect<2>> AdcpBiasType;
typedef ukfom::mtkwrap<MTK::vect<1>> DensityType;
typedef ukfom::mtkwrap<MTK::vect<2>> Translation2DType;

MTK_BUILD_MANIFOLD(
    PoseState,
    ((TranslationType, position))  // position of IMU in navigation frame
    ((RotationType, orientation))  // orientation of IMU in navigation frame
    ((VelocityType, velocity))     // velocity of IMU in navigation frame
    ((AccelerationType,
      acceleration))          // acceleration of IMU in navigation frame
    ((BiasType, bias_gyro))   // gyroscope bias states
    ((BiasType, bias_acc))    // acceleration bias states
    ((GravityType, gravity))  // local gravity, to refine the WGS-84 ellipsoid
                              // earth gravity model
    ((InertiaType, inertia))  // [xx, xy, xψ; yx, yy, yψ; ψx, ψy, ψψ] components
                              // of the inertia matrix in column-major order. ψ
                              // is the rotation around the z-axis.
    ((LinDampingType,
      lin_damping))  // [xx, xy, xψ; yx, yy, yψ; ψx, ψy, ψψ] components of the
                     // linear damping matrix in column-major order. ψ is the
                     // rotation around the z-axis.
    ((QuadDampingType,
      quad_damping))  // [xx, xy, xψ; yx, yy, yψ; ψx, ψy, ψψ] components of the
                      // quadratic damping matrix in column-major order. ψ is
                      // the rotation around the z-axis.
    ((WaterVelocityType,
      water_velocity))  // Water current velocity surrounding the vehicle, in
                        // the North/East directions
    ((WaterVelocityType,
      water_velocity_below))     // Water current velocity below the vehicle, in
                                 // the North/East directions
    ((AdcpBiasType, bias_adcp))  // ADCP bias states
    ((DensityType,
      water_density))  // water density in kg/m³
                       //((Translation2DType, delayed_position)) // delayed xy
                       // position of IMU in navigation frame
)

MTK_BUILD_MANIFOLD(
    SimplePoseState,
    ((TranslationType, position))  // position of IMU in navigation frame
    ((RotationType, orientation))  // orientation of IMU in navigation frame
    ((VelocityType, velocity))     // velocity of IMU in navigation frame
    ((AccelerationType,
      acceleration))          // acceleration of IMU in navigation frame
    ((BiasType, bias_gyro))   // gyroscope bias states
    ((BiasType, bias_acc))    // acceleration bias states
    ((GravityType, gravity))  // local gravity,
    //                      to refine the WGS-84 ellipsoid earth gravity model
    //                      ((WaterVelocityType, water_velocity))       // Water
    //                      current velocity surrounding the vehicle, in the
    //                      North/East directions
    //                      ((WaterVelocityType, water_velocity_below)) // Water
    //                      current velocity below the vehicle, in the
    //                      North/East directions
    //                      ((AdcpBiasType, bias_adcp))                 // ADCP
    //                      bias states
    ((DensityType, water_density))  // water density in kg/m³
)

}  // namespace uwv_kalman_filters

#endif
