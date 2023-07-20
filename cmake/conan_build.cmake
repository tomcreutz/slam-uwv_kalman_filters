list(APPEND CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}")

find_package(Eigen3 REQUIRED)
list(APPEND uwv_kalman_filters_EXTRA_LIBRARIES Eigen3::Eigen)
message(STATUS "Eigen3_LIBRARIES: Eigen3::Eigen")
find_package(pose_estimation REQUIRED)
list(APPEND uwv_kalman_filters_EXTRA_LIBRARIES pose_estimation::pose_estimation)
message(STATUS "pose_estimation_LIBRARIES: pose_estimation::pose_estimation")
find_package(mtk REQUIRED)
list(APPEND uwv_kalman_filters_EXTRA_LIBRARIES mtk::mtk)
message(STATUS "mtk_LIBRARIES: mtk::mtk")

set(uwv_kalman_filters_LIB_DESTINATION lib)
set(uwv_kalman_filters_INCLUDE_DESTINATION include)
set(uwv_kalman_filters_BIN_DESTINATION bin)

mark_as_advanced(
  uwv_kalman_filters_EXTRA_LIBRARIES
  uwv_kalman_filters_LIB_DESTINATION
  uwv_kalman_filters_INCLUDE_DESTINATION
  uwv_kalman_filters_BIN_DESTINATION)

macro(export_uwv_kalman_filters_package)
  install(EXPORT ${PROJECT_NAME}Targets
    FILE "${PROJECT_NAME}Targets.cmake"
    DESTINATION "${uwv_kalman_filters_LIB_DESTINATION}/cmake/${PROJECT_NAME}"
    NAMESPACE uwv_kalman_filters::
  )
  export(PACKAGE ${PROJECT_NAME})

  include(CMakePackageConfigHelpers)

  configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION "${uwv_kalman_filters_LIB_DESTINATION}/cmake/${PROJECT_NAME}"
  )

  install(
    FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    DESTINATION "${uwv_kalman_filters_LIB_DESTINATION}/cmake/${PROJECT_NAME}"
  )
endmacro()