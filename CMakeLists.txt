cmake_minimum_required(VERSION 3.14)
project(eight_point_estimator)

set( CMAKE_CXX_STANDARD 17 )

set( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )

find_package( Eigen3 3.3 REQUIRED )
find_package( OpenCV 4.1.0 REQUIRED )

add_subdirectory( vslam_utils )
add_subdirectory( tests )
