# ---- Add the subdirectory cmake ----
set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(Eigen3 REQUIRED)
find_package(pose_estimation REQUIRED)
find_package(mtk REQUIRED)

find_package(catkin REQUIRED COMPONENTS roslib)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${uwv_kalman_filters_LIBRARY}
  CATKIN_DEPENDS roslib)

set(uwv_kalman_filters_EXTRA_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})

set(uwv_kalman_filters_EXTRA_LIBRARIES
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  pose_estimation::pose_estimation
  mtk::mtk
)

set(uwv_kalman_filters_LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
set(uwv_kalman_filters_INCLUDE_DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
set(uwv_kalman_filters_BIN_DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

mark_as_advanced(
  uwv_kalman_filters_EXTRA_LIBRARIES
  uwv_kalman_filters_EXTRA_INCLUDE_DIRS
  uwv_kalman_filters_LIB_DESTINATION
  uwv_kalman_filters_INCLUDE_DESTINATION
  uwv_kalman_filters_BIN_DESTINATION)

macro(export_uwv_kalman_filters_package)
  # do nothing
endmacro()