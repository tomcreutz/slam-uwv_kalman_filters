# ---- Add the subdirectory cmake ----
set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(Eigen3 REQUIRED)
find_package(pose_estimation REQUIRED)
find_package(mtk REQUIRED)

find_package(ament_index_cpp REQUIRED)

set(uwv_kalman_filters_EXTRA_LIBRARIES
  $<BUILD_INTERFACE:ament_index_cpp::ament_index_cpp>
  ${pose_estimation_LIBRARIES}
  Eigen3::Eigen
  mtk::mtk
)

set(uwv_kalman_filters_EXTRA_INCLUDE_DIRS
  ${pose_estimation_INCLUDE_DIRS}
)

ament_export_dependencies(ament_index_cpp)

set(uwv_kalman_filters_LIB_DESTINATION lib)
set(uwv_kalman_filters_INCLUDE_DESTINATION include)
set(uwv_kalman_filters_BIN_DESTINATION bin)

mark_as_advanced(
  uwv_kalman_filters_EXTRA_LIBRARIES
  uwv_kalman_filters_LIB_DESTINATION
  uwv_kalman_filters_INCLUDE_DESTINATION
  uwv_kalman_filters_BIN_DESTINATION)

macro(export_uwv_kalman_filters_package)
  ament_export_include_directories(include)
  ament_export_libraries(${uwv_kalman_filters_LIBRARY})
  ament_package()
endmacro()