cmake_minimum_required(VERSION 3.8)
project(uwv_kalman_filters VERSION "0.1.0")

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

set(uwv_kalman_filters_LIBRARY ${PROJECT_NAME})

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE
    STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

include(GNUInstallDirs)
include(CMakeDependentOption)
include(CMakePackageConfigHelpers)

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# include(AddMTK)
# add_subdirectory(extern/mtk)
option(PACKAGE_TESTS "Build the tests" ON)

find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
  add_definitions(-DUSING_ROS2)
  message(STATUS "------------------------------------------")
  message(STATUS "BehaviourTree is being built using AMENT.")
  message(STATUS "------------------------------------------")
  include(cmake/ament_build.cmake)

elseif(CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)
  add_definitions(-DUSING_ROS)
  message(STATUS "------------------------------------------")
  message(STATUS "BehaviourTree is being built using CATKIN.")
  message(STATUS "------------------------------------------")
  include(cmake/catkin_build.cmake)
  set(catkin_FOUND TRUE)
else()
  message(STATUS "------------------------------------------")
  message(STATUS "BehaviourTree is being built with conan.")
  message(STATUS "------------------------------------------")
  include(cmake/conan_build.cmake)
endif()

# pkg_check_modules(mtk REQUIRED mtk ukfom)
list(APPEND uwv_kalman_filters_SOURCE
  src/SimplePoseUKF.cpp

  # src/PoseUKF.cpp
  # src/VelocityUKF.cpp
  # src/IndirectPoseUKF.cpp
  # src/BottomUKF.cpp
)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON) # ? - https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/CMakeLists.txt#L147
add_library(${uwv_kalman_filters_LIBRARY} SHARED
  ${uwv_kalman_filters_SOURCE}
)

target_link_libraries(${uwv_kalman_filters_LIBRARY}
  PRIVATE
  ${CMAKE_DL_LIBS}
  ${uwv_kalman_filters_EXTRA_LIBRARIES}
)

target_include_directories(${uwv_kalman_filters_LIBRARY}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE
  ${uwv_kalman_filters_EXTRA_INCLUDE_DIRS}
)

target_compile_definitions(${uwv_kalman_filters_LIBRARY} PUBLIC uwv_kalman_filters_LIBRARY_VERSION="${CMAKE_PROJECT_VERSION}")

target_compile_features(${uwv_kalman_filters_LIBRARY} PUBLIC cxx_std_17)

INSTALL(TARGETS ${uwv_kalman_filters_LIBRARY}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${uwv_kalman_filters_LIB_DESTINATION}
  LIBRARY DESTINATION ${uwv_kalman_filters_LIB_DESTINATION}
  RUNTIME DESTINATION ${uwv_kalman_filters_BIN_DESTINATION}
  INCLUDES DESTINATION ${uwv_kalman_filters_INCLUDE_DESTINATION}
)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${uwv_kalman_filters_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h*")

export_uwv_kalman_filters_package()