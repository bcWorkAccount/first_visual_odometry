cmake_minimum_required(VERSION 3.5)
project(undistort)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE RELEASE)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
${Sophus_INCLUDE_DIRS})

include_directories("/usr/include/eigen3/")


add_executable(disparity disparity_1.cpp)
target_link_libraries(disparity ${Pangolin_LIBRARIES} ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES})
