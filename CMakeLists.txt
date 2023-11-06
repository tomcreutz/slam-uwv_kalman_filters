cmake_minimum_required(VERSION 3.5)
project(uwv_kalman_filters VERSION "0.1.0")

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE
    STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(pose_estimation REQUIRED)
find_package(mtk REQUIRED)

add_library(${PROJECT_NAME} SHARED
src/PoseUKF.cpp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "AMENT_INDEX_CPP_BUILDING_DLL")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
  ${pose_estimation_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME}
  PRIVATE
  Eigen3::Eigen
  mtk::mtk
  ${pose_estimation_LIBRARIES}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})

# Todo(tcreutz): Tests
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # prevent cppcheck from being found, since it fails to process the file utest.cpp
#   set(ament_cmake_cppcheck_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

#   ament_add_gtest(${PROJECT_NAME}_utest test/utest.cpp)
#   if(TARGET ${PROJECT_NAME}_utest)
#     target_include_directories(${PROJECT_NAME}_utest PUBLIC include)
#     target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME})
#   endif()
# endif()

ament_package()

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)