cmake_minimum_required(VERSION 2.8)
project( FistVO )

# the build type
set( CMAKE_BUILD_TYPE "Release" )

# the execute  output to "./bin". lib output to "./lib"
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Check C++11  support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)

# thirdparty libs
find_package( OpenCV 3.1 REQUIRED)
find_package( Eigen3 3.1.0 REQUIRED)
find_package( Pangolin REQUIRED)
find_package( Glog REQUIRED )
find_package( PCL REQUIRED COMPONENT common io )
find_package( Sophus REQUIRED)
find_package( Ceres REQUIRED  )
find_package( G2O REQUIRED )


SET (G2O_LIBRARIES csparse cxsparse g2o_csparse_extension g2o_cli g2o_core g2o_interface g2o_parser g2o_solver_cholmod
     g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp
    g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
)

add_definitions( ${PCL_DEFINITIONS} )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# thirdparty
set( third_party_libs
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${BLAS_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${GLOG_LIBRARY}
    ${PCL_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${CERES_LIBRARIES}
    ${G2O_LIBRARIES}
)

# subdirectories
include_directories(
    visual_odometry/include
    common/include
    backend/include
)

# add the sub directory
add_subdirectory( common )
add_subdirectory( visual_odometry)
set( fvo_all_libs  fvo_vo_libs  fvo_common_libs )

# build examples in the  sub directory "examples"
add_subdirectory( examples )