cmake_minimum_required(VERSION 3.0.0)
project(robot_pose_fusion)

# Compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED PATHS /usr/include/eigen3)

if(EIGEN3_FOUND)
  message("found eigen3")
endif()



## Build ##
set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

# ekf
add_library(robot_pose_ekf SHARED
  src/robot_pose_ekf.cpp
  src/median_filter.cpp
)

# run app
add_executable(fusion fusion.cpp)
target_link_libraries(fusion
  robot_pose_ekf
)
